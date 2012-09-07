"""autogenerated by genpy from snazzy_msgs/ProcessCloudRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg
import sensor_msgs.msg

class ProcessCloudRequest(genpy.Message):
  _md5sum = "fbcba72803c7ee38c4e565d0ce4aa21b"
  _type = "snazzy_msgs/ProcessCloudRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """sensor_msgs/PointCloud2 cloud_in
std_msgs/String str_args
std_msgs/Float32 float_args

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: std_msgs/Float32
float32 data
"""
  __slots__ = ['cloud_in','str_args','float_args']
  _slot_types = ['sensor_msgs/PointCloud2','std_msgs/String','std_msgs/Float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       cloud_in,str_args,float_args

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ProcessCloudRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.cloud_in is None:
        self.cloud_in = sensor_msgs.msg.PointCloud2()
      if self.str_args is None:
        self.str_args = std_msgs.msg.String()
      if self.float_args is None:
        self.float_args = std_msgs.msg.Float32()
    else:
      self.cloud_in = sensor_msgs.msg.PointCloud2()
      self.str_args = std_msgs.msg.String()
      self.float_args = std_msgs.msg.Float32()

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
      buff.write(_struct_3I.pack(_x.cloud_in.header.seq, _x.cloud_in.header.stamp.secs, _x.cloud_in.header.stamp.nsecs))
      _x = self.cloud_in.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.cloud_in.height, _x.cloud_in.width))
      length = len(self.cloud_in.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.cloud_in.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_IBI.pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_struct_B2I.pack(_x.cloud_in.is_bigendian, _x.cloud_in.point_step, _x.cloud_in.row_step))
      _x = self.cloud_in.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.cloud_in.is_dense))
      _x = self.str_args.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_f.pack(self.float_args.data))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.cloud_in is None:
        self.cloud_in = sensor_msgs.msg.PointCloud2()
      if self.str_args is None:
        self.str_args = std_msgs.msg.String()
      if self.float_args is None:
        self.float_args = std_msgs.msg.Float32()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.cloud_in.header.seq, _x.cloud_in.header.stamp.secs, _x.cloud_in.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cloud_in.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.cloud_in.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.cloud_in.height, _x.cloud_in.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.cloud_in.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _struct_IBI.unpack(str[start:end])
        self.cloud_in.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.cloud_in.is_bigendian, _x.cloud_in.point_step, _x.cloud_in.row_step,) = _struct_B2I.unpack(str[start:end])
      self.cloud_in.is_bigendian = bool(self.cloud_in.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cloud_in.data = str[start:end].decode('utf-8')
      else:
        self.cloud_in.data = str[start:end]
      start = end
      end += 1
      (self.cloud_in.is_dense,) = _struct_B.unpack(str[start:end])
      self.cloud_in.is_dense = bool(self.cloud_in.is_dense)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.str_args.data = str[start:end].decode('utf-8')
      else:
        self.str_args.data = str[start:end]
      start = end
      end += 4
      (self.float_args.data,) = _struct_f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.cloud_in.header.seq, _x.cloud_in.header.stamp.secs, _x.cloud_in.header.stamp.nsecs))
      _x = self.cloud_in.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.cloud_in.height, _x.cloud_in.width))
      length = len(self.cloud_in.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.cloud_in.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_IBI.pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_struct_B2I.pack(_x.cloud_in.is_bigendian, _x.cloud_in.point_step, _x.cloud_in.row_step))
      _x = self.cloud_in.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.cloud_in.is_dense))
      _x = self.str_args.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_f.pack(self.float_args.data))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.cloud_in is None:
        self.cloud_in = sensor_msgs.msg.PointCloud2()
      if self.str_args is None:
        self.str_args = std_msgs.msg.String()
      if self.float_args is None:
        self.float_args = std_msgs.msg.Float32()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.cloud_in.header.seq, _x.cloud_in.header.stamp.secs, _x.cloud_in.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cloud_in.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.cloud_in.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.cloud_in.height, _x.cloud_in.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.cloud_in.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _struct_IBI.unpack(str[start:end])
        self.cloud_in.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.cloud_in.is_bigendian, _x.cloud_in.point_step, _x.cloud_in.row_step,) = _struct_B2I.unpack(str[start:end])
      self.cloud_in.is_bigendian = bool(self.cloud_in.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cloud_in.data = str[start:end].decode('utf-8')
      else:
        self.cloud_in.data = str[start:end]
      start = end
      end += 1
      (self.cloud_in.is_dense,) = _struct_B.unpack(str[start:end])
      self.cloud_in.is_dense = bool(self.cloud_in.is_dense)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.str_args.data = str[start:end].decode('utf-8')
      else:
        self.str_args.data = str[start:end]
      start = end
      end += 4
      (self.float_args.data,) = _struct_f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_IBI = struct.Struct("<IBI")
_struct_B = struct.Struct("<B")
_struct_f = struct.Struct("<f")
_struct_3I = struct.Struct("<3I")
_struct_B2I = struct.Struct("<B2I")
_struct_2I = struct.Struct("<2I")
"""autogenerated by genpy from snazzy_msgs/ProcessCloudResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg
import sensor_msgs.msg

class ProcessCloudResponse(genpy.Message):
  _md5sum = "d638895a709be2cef85df359cc39f0dc"
  _type = "snazzy_msgs/ProcessCloudResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """sensor_msgs/PointCloud2 cloud_out


================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

"""
  __slots__ = ['cloud_out']
  _slot_types = ['sensor_msgs/PointCloud2']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       cloud_out

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ProcessCloudResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.cloud_out is None:
        self.cloud_out = sensor_msgs.msg.PointCloud2()
    else:
      self.cloud_out = sensor_msgs.msg.PointCloud2()

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
      buff.write(_struct_3I.pack(_x.cloud_out.header.seq, _x.cloud_out.header.stamp.secs, _x.cloud_out.header.stamp.nsecs))
      _x = self.cloud_out.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.cloud_out.height, _x.cloud_out.width))
      length = len(self.cloud_out.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.cloud_out.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_IBI.pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_struct_B2I.pack(_x.cloud_out.is_bigendian, _x.cloud_out.point_step, _x.cloud_out.row_step))
      _x = self.cloud_out.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.cloud_out.is_dense))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.cloud_out is None:
        self.cloud_out = sensor_msgs.msg.PointCloud2()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.cloud_out.header.seq, _x.cloud_out.header.stamp.secs, _x.cloud_out.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cloud_out.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.cloud_out.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.cloud_out.height, _x.cloud_out.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.cloud_out.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _struct_IBI.unpack(str[start:end])
        self.cloud_out.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.cloud_out.is_bigendian, _x.cloud_out.point_step, _x.cloud_out.row_step,) = _struct_B2I.unpack(str[start:end])
      self.cloud_out.is_bigendian = bool(self.cloud_out.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cloud_out.data = str[start:end].decode('utf-8')
      else:
        self.cloud_out.data = str[start:end]
      start = end
      end += 1
      (self.cloud_out.is_dense,) = _struct_B.unpack(str[start:end])
      self.cloud_out.is_dense = bool(self.cloud_out.is_dense)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.cloud_out.header.seq, _x.cloud_out.header.stamp.secs, _x.cloud_out.header.stamp.nsecs))
      _x = self.cloud_out.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.cloud_out.height, _x.cloud_out.width))
      length = len(self.cloud_out.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.cloud_out.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_IBI.pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_struct_B2I.pack(_x.cloud_out.is_bigendian, _x.cloud_out.point_step, _x.cloud_out.row_step))
      _x = self.cloud_out.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.cloud_out.is_dense))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.cloud_out is None:
        self.cloud_out = sensor_msgs.msg.PointCloud2()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.cloud_out.header.seq, _x.cloud_out.header.stamp.secs, _x.cloud_out.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cloud_out.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.cloud_out.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.cloud_out.height, _x.cloud_out.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.cloud_out.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _struct_IBI.unpack(str[start:end])
        self.cloud_out.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.cloud_out.is_bigendian, _x.cloud_out.point_step, _x.cloud_out.row_step,) = _struct_B2I.unpack(str[start:end])
      self.cloud_out.is_bigendian = bool(self.cloud_out.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cloud_out.data = str[start:end].decode('utf-8')
      else:
        self.cloud_out.data = str[start:end]
      start = end
      end += 1
      (self.cloud_out.is_dense,) = _struct_B.unpack(str[start:end])
      self.cloud_out.is_dense = bool(self.cloud_out.is_dense)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_IBI = struct.Struct("<IBI")
_struct_3I = struct.Struct("<3I")
_struct_B = struct.Struct("<B")
_struct_2I = struct.Struct("<2I")
_struct_B2I = struct.Struct("<B2I")
class ProcessCloud(object):
  _type          = 'snazzy_msgs/ProcessCloud'
  _md5sum = '62372c233a3d22e82d88cc898c7261e6'
  _request_class  = ProcessCloudRequest
  _response_class = ProcessCloudResponse