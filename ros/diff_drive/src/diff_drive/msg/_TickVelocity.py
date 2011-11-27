"""autogenerated by genmsg_py from TickVelocity.msg. Do not edit."""
import roslib.message
import struct


class TickVelocity(roslib.message.Message):
  _md5sum = "c303165597398c26cce74324035e7fc0"
  _type = "diff_drive/TickVelocity"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int16   linear_ticks_sec
int16   angular_ticks_sec

"""
  __slots__ = ['linear_ticks_sec','angular_ticks_sec']
  _slot_types = ['int16','int16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       linear_ticks_sec,angular_ticks_sec
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(TickVelocity, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.linear_ticks_sec is None:
        self.linear_ticks_sec = 0
      if self.angular_ticks_sec is None:
        self.angular_ticks_sec = 0
    else:
      self.linear_ticks_sec = 0
      self.angular_ticks_sec = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_2h.pack(_x.linear_ticks_sec, _x.angular_ticks_sec))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 4
      (_x.linear_ticks_sec, _x.angular_ticks_sec,) = _struct_2h.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_2h.pack(_x.linear_ticks_sec, _x.angular_ticks_sec))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 4
      (_x.linear_ticks_sec, _x.angular_ticks_sec,) = _struct_2h.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_2h = struct.Struct("<2h")