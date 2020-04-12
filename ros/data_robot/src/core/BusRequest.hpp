// BusRequest.hpp

#include <pthread.h>
#include <time.h>
#include <inttypes.h>

#ifndef GUARD_BusRequest
#define GUARD_BusRequest

namespace data_robot_core
{
enum RequestType_T
{
  REQUEST_INVALID = 0,
  REQUEST_WRITE,
  REQUEST_READ
};

/** This class stores the address and data for a bus transaction.
 *
 *  There are two main properties that determine the behavior of the class. 
 *  The request can be lockable and blockable using pthread mutexes and 
 *  conditionals. Calling @ref Block() the request will halt the current thread until
 *  @ref Unblock() is called in another thread.
 */
class BusRequest
{
public:
  BusRequest( bool is_blockable = true, bool is_lockable = true );

  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~BusRequest();

  virtual bool  IsLockable() const  { return _p_lock_mutex ? true:false; }
  virtual bool  IsLocked() const    { return _is_locked; }
  virtual void  Lock();
  virtual void  Unlock();
  virtual int   TryLock();

  virtual bool  IsBlockable() const { return _p_block_cond ? true:false; }
  virtual bool  IsBlocked() const   { return _is_blocked; }
  virtual void  Block();
  virtual void  Unblock();
  
  /** Returns the current request type. */
  virtual RequestType_T GetRequestType() const  { return _request_type; }
  virtual void  SetRequestType( RequestType_T new_type );
  virtual bool  GetRequestComplete() const { return _request_complete; }
  virtual void  SetRequestComplete( bool request_complete ) { _request_complete = request_complete; }

  /** Returns the current size of the address buffer in bytes. */
  virtual unsigned int  GetAddressBufferSize() const  { return _address_size; }
  virtual void          SetAddressBufferSize( unsigned int new_size );
  virtual unsigned int  GetAddress( uint8_t* dest, unsigned int max_bytes ) const;
  virtual int           SetAddress( const uint8_t* src, unsigned int num_bytes );
  virtual uint8_t*      GetAddressBuffer()  const { return _p_address_buffer; }
 
  /** Returns the size of the data buffer in bytes. */
  virtual unsigned int  GetDataBufferSize() const { return _data_size; }
  virtual void          SetDataBufferSize( unsigned int new_size );
  virtual int           GetData( uint8_t* dest, unsigned int max_bytes ) const;
  virtual int           SetData( const uint8_t* src, unsigned int num_bytes );
  virtual uint8_t*      GetDataBuffer() const { return _p_data_buffer; }

  /** Returns the time last set. */
  virtual struct timespec GetTimeStamp() const { return _time; }
  virtual void  SetTimeStamp( struct timespec new_time );

private:
  // Disable the copy constructor and assignment operator because I don't know
  // how to deal with mutex's and cond's in a sane way in the destructor.
  BusRequest(const BusRequest&);     // do not give these a body
  BusRequest& operator = (const BusRequest&);
  
  bool _is_locked;
  pthread_mutex_t  *_p_lock_mutex;

  bool _is_blocked; 
  pthread_cond_t   *_p_block_cond;                   

  RequestType_T _request_type;
  bool      _request_complete;

  uint8_t   _address_bytes;
  uint8_t   _address_size;
  uint8_t  *_p_address_buffer;

  uint32_t  _data_bytes;
  uint32_t  _data_size;
  uint8_t  *_p_data_buffer;

  struct timespec _time;
};
}
 
#endif /* GUARD_BusRequest */
