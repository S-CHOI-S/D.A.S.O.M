# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from crazyflie_driver/GoToRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy
import geometry_msgs.msg

class GoToRequest(genpy.Message):
  _md5sum = "82856b48a972d6af023d961a655bcf75"
  _type = "crazyflie_driver/GoToRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint8 groupMask
bool relative
geometry_msgs/Point goal
float32 yaw # deg
duration duration

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
  __slots__ = ['groupMask','relative','goal','yaw','duration']
  _slot_types = ['uint8','bool','geometry_msgs/Point','float32','duration']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       groupMask,relative,goal,yaw,duration

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GoToRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.groupMask is None:
        self.groupMask = 0
      if self.relative is None:
        self.relative = False
      if self.goal is None:
        self.goal = geometry_msgs.msg.Point()
      if self.yaw is None:
        self.yaw = 0.
      if self.duration is None:
        self.duration = genpy.Duration()
    else:
      self.groupMask = 0
      self.relative = False
      self.goal = geometry_msgs.msg.Point()
      self.yaw = 0.
      self.duration = genpy.Duration()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_2B3df2i().pack(_x.groupMask, _x.relative, _x.goal.x, _x.goal.y, _x.goal.z, _x.yaw, _x.duration.secs, _x.duration.nsecs))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.goal is None:
        self.goal = geometry_msgs.msg.Point()
      if self.duration is None:
        self.duration = genpy.Duration()
      end = 0
      _x = self
      start = end
      end += 38
      (_x.groupMask, _x.relative, _x.goal.x, _x.goal.y, _x.goal.z, _x.yaw, _x.duration.secs, _x.duration.nsecs,) = _get_struct_2B3df2i().unpack(str[start:end])
      self.relative = bool(self.relative)
      self.duration.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2B3df2i().pack(_x.groupMask, _x.relative, _x.goal.x, _x.goal.y, _x.goal.z, _x.yaw, _x.duration.secs, _x.duration.nsecs))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.goal is None:
        self.goal = geometry_msgs.msg.Point()
      if self.duration is None:
        self.duration = genpy.Duration()
      end = 0
      _x = self
      start = end
      end += 38
      (_x.groupMask, _x.relative, _x.goal.x, _x.goal.y, _x.goal.z, _x.yaw, _x.duration.secs, _x.duration.nsecs,) = _get_struct_2B3df2i().unpack(str[start:end])
      self.relative = bool(self.relative)
      self.duration.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2B3df2i = None
def _get_struct_2B3df2i():
    global _struct_2B3df2i
    if _struct_2B3df2i is None:
        _struct_2B3df2i = struct.Struct("<2B3df2i")
    return _struct_2B3df2i
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from crazyflie_driver/GoToResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class GoToResponse(genpy.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "crazyflie_driver/GoToResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
"""
  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GoToResponse, self).__init__(*args, **kwds)

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
class GoTo(object):
  _type          = 'crazyflie_driver/GoTo'
  _md5sum = '82856b48a972d6af023d961a655bcf75'
  _request_class  = GoToRequest
  _response_class = GoToResponse
