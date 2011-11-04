// BusRequest.hpp

#include "BusRequest.hpp"

namespace data_robot_core
{
BusRequest::BusRequest( bool is_blockable, bool is_lockable )
          : _address_size(0),
            _address_bytes(0),
            _p_address_buffer(NULL),
            _data_size(0),
            _data_bytes(0),
            _p_data_buffer(NULL),
            _is_locked(false),
            _is_blocked(false),
            _request_complete(false)
{
  if( is_blockable	)
  {
    is_lockable = true;
  }

  if( is_lockable )
  {
    _p_lock_mutex = new pthread_mutex_t;

    pthread_mutex_init( _p_lock_mutex, NULL );
  }
  else
  {
    _p_lock_mutex = NULL;
  }

  if( is_blockable )
  {
    _p_block_cond = new pthread_cond_t;

    pthread_cond_init( _p_block_cond, NULL );
  }
  else
  {
    _p_block_cond = NULL;
  }
}

// Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
BusRequest::~BusRequest() 
{
  if ( _p_lock_mutex != NULL ) 
  {
    pthread_mutex_destroy( _p_lock_mutex );
    delete _p_lock_mutex;
  }

  if ( _p_block_cond != NULL )
  {
    pthread_cond_destroy( _p_block_cond );
    delete _p_block_cond;
  }

  if ( _p_address_buffer != NULL )
  {
    delete [] _p_address_buffer;
  }

  if ( _p_data_buffer != NULL )
  {
    delete [] _p_data_buffer;
  }
}

/** Tries to aquire an internal mutex, blocks until it is available.
 */
void  BusRequest::Lock()
{
  if ( _p_lock_mutex != NULL )
  {
    pthread_mutex_lock( _p_lock_mutex );
    _is_locked = true;
  }
}

/** Releases the lock on the internal mutex.
 */
void  BusRequest::Unlock()
{
  if ( _p_lock_mutex != NULL )
  {
    pthread_mutex_unlock( _p_lock_mutex );
    _is_locked = false;
  }
}

/** Non-blocking attempt to lock the internal mutex.
 *  @retval 0  Lock successfully aquired, or not lockable.
 *  @retval Other Lock not aquired.
 */
int   BusRequest::TryLock()
{
  if ( _p_lock_mutex != NULL )
  {
    _is_locked = !pthread_mutex_trylock( _p_lock_mutex );
    return !_is_locked;
  }

  return 0;
}

/** Blocks the calling thread until a different thread calls Unblock.
 *
 *  Aquires the mutex lock if we do not have it.
 */
void  BusRequest::Block()
{
  bool externally_locked = true;

  if ( _p_block_cond != NULL )
  {
    if ( !_is_locked )
    {
      externally_locked = false;
      Lock();
    }

    _is_blocked = true;
    _is_locked = false; // pthread_cond_wait will unlock the mutex

    pthread_cond_wait( _p_block_cond, _p_lock_mutex );
   
    // We have been released!
    _is_blocked = false;

    // If we had to lock ourself then unlock
    if ( ! externally_locked )
    {
      Unlock();
    }
  }
}

/** If @ref Block() had previously been called, calling this function will 
 *  allow the previous thread to run.
 */
void  BusRequest::Unblock()
{
  bool externally_locked = true;

  if ( _p_block_cond != NULL )
  {
    if ( _is_blocked )
    {
      if ( !_is_locked )
      {
        externally_locked = false;
        Lock();
      }

      pthread_cond_signal( _p_block_cond );

      // If we had to lock ourself then unlock
      if ( ! externally_locked )
      {
        Unlock();
      }
  	}
  }
}

/** Returns the currently set Request Type.
 */
void  BusRequest::SetRequestType( RequestType_T new_type )
{
  _request_type = new_type;
}

/** Sets the address buffer size in bytes and allocates a new buffer.
 */
void  BusRequest::SetAddressBufferSize( int new_size )
{
  if ( new_size != _address_size )
  {
    if ( _p_address_buffer != NULL )
    {
      delete [] _p_address_buffer;
    }

    _p_address_buffer = new uint8_t [ new_size ];
    _address_size = new_size;
    _address_bytes = 0;
  }
}

/** Copies the contents of the address buffer into @p dest.
 *  @returns the number of bytes copied.
 */
int   BusRequest::GetAddress( uint8_t* dest, int max_bytes ) const
{
  if ( max_bytes > _address_bytes )
  {   
    max_bytes = _address_bytes;
  }

  for ( int i = 0; i < max_bytes; i++ )
  {
    dest[i] = _p_address_buffer[i];
  }

  return max_bytes;
}

/** Copies the contents of src into the address buffer, re-allocating the buffer 
 *  if it is not big enough.
 */
int   BusRequest::SetAddress( const uint8_t* src, int num_bytes )
{
  if ( num_bytes > _address_size )
  {
    SetAddressBufferSize( num_bytes );  
  }

  _address_bytes = num_bytes;

  for ( int i = 0; i < _address_bytes; i++ )
  {
    _p_address_buffer[i] = src[i];
  }

  return _address_bytes;
}

/** Sets the data buffer size in bytes and allocates a new buffer.
 */
void  BusRequest::SetDataBufferSize( int new_size )
{
  if ( new_size != _data_size )
  {
    if ( _p_data_buffer != NULL )
    {
      delete [] _p_data_buffer;
    }

    _p_data_buffer = new uint8_t [ new_size ];
    _data_size = new_size;
    _data_bytes = 0;
  }
}

/** Copies the contents of the data buffer into @p dest.
 *  @returns the number of bytes copied.
 */
int   BusRequest::GetData( uint8_t* dest, int max_bytes ) const
{
  if ( max_bytes > _data_bytes )
  {   
    max_bytes = _data_bytes;
  }

  for ( int i = 0; i < max_bytes; i++ )
  {
    dest[i] = _p_data_buffer[i];
  }

  return max_bytes;
}

/** Copies the contents of src into the data buffer, re-allocating the buffer 
 *  if it is not big enough.
 */
int   BusRequest::SetData( const uint8_t* src, int num_bytes )
{
  if ( num_bytes > _data_size )
  {
    SetDataBufferSize( num_bytes );  
  }

  _data_bytes = num_bytes;

  for ( int i = 0; i < _data_bytes; i++ )
  {
    _p_data_buffer[i] = src[i];
  }

  return _data_bytes;
}

/** Sets the time associated with the request, often set when the request is
 *  completed.
 */
void  BusRequest::SetTime( struct timespec new_time )
{
  _time = new_time;	
}
}
