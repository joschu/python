; Auto-generated. Do not edit!


(cl:in-package snazzy_msgs-srv)


;//! \htmlinclude ProcessCloud-request.msg.html

(cl:defclass <ProcessCloud-request> (roslisp-msg-protocol:ros-message)
  ((cloud_in
    :reader cloud_in
    :initarg :cloud_in
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (str_args
    :reader str_args
    :initarg :str_args
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (float_args
    :reader float_args
    :initarg :float_args
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32)))
)

(cl:defclass ProcessCloud-request (<ProcessCloud-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProcessCloud-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProcessCloud-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name snazzy_msgs-srv:<ProcessCloud-request> is deprecated: use snazzy_msgs-srv:ProcessCloud-request instead.")))

(cl:ensure-generic-function 'cloud_in-val :lambda-list '(m))
(cl:defmethod cloud_in-val ((m <ProcessCloud-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snazzy_msgs-srv:cloud_in-val is deprecated.  Use snazzy_msgs-srv:cloud_in instead.")
  (cloud_in m))

(cl:ensure-generic-function 'str_args-val :lambda-list '(m))
(cl:defmethod str_args-val ((m <ProcessCloud-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snazzy_msgs-srv:str_args-val is deprecated.  Use snazzy_msgs-srv:str_args instead.")
  (str_args m))

(cl:ensure-generic-function 'float_args-val :lambda-list '(m))
(cl:defmethod float_args-val ((m <ProcessCloud-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snazzy_msgs-srv:float_args-val is deprecated.  Use snazzy_msgs-srv:float_args instead.")
  (float_args m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProcessCloud-request>) ostream)
  "Serializes a message object of type '<ProcessCloud-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_in) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'str_args) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'float_args) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProcessCloud-request>) istream)
  "Deserializes a message object of type '<ProcessCloud-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_in) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'str_args) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'float_args) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProcessCloud-request>)))
  "Returns string type for a service object of type '<ProcessCloud-request>"
  "snazzy_msgs/ProcessCloudRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProcessCloud-request)))
  "Returns string type for a service object of type 'ProcessCloud-request"
  "snazzy_msgs/ProcessCloudRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProcessCloud-request>)))
  "Returns md5sum for a message object of type '<ProcessCloud-request>"
  "62372c233a3d22e82d88cc898c7261e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProcessCloud-request)))
  "Returns md5sum for a message object of type 'ProcessCloud-request"
  "62372c233a3d22e82d88cc898c7261e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProcessCloud-request>)))
  "Returns full string definition for message of type '<ProcessCloud-request>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_in~%std_msgs/String str_args~%std_msgs/Float32 float_args~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProcessCloud-request)))
  "Returns full string definition for message of type 'ProcessCloud-request"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_in~%std_msgs/String str_args~%std_msgs/Float32 float_args~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProcessCloud-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_in))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'str_args))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'float_args))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProcessCloud-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ProcessCloud-request
    (cl:cons ':cloud_in (cloud_in msg))
    (cl:cons ':str_args (str_args msg))
    (cl:cons ':float_args (float_args msg))
))
;//! \htmlinclude ProcessCloud-response.msg.html

(cl:defclass <ProcessCloud-response> (roslisp-msg-protocol:ros-message)
  ((cloud_out
    :reader cloud_out
    :initarg :cloud_out
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass ProcessCloud-response (<ProcessCloud-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProcessCloud-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProcessCloud-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name snazzy_msgs-srv:<ProcessCloud-response> is deprecated: use snazzy_msgs-srv:ProcessCloud-response instead.")))

(cl:ensure-generic-function 'cloud_out-val :lambda-list '(m))
(cl:defmethod cloud_out-val ((m <ProcessCloud-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snazzy_msgs-srv:cloud_out-val is deprecated.  Use snazzy_msgs-srv:cloud_out instead.")
  (cloud_out m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProcessCloud-response>) ostream)
  "Serializes a message object of type '<ProcessCloud-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_out) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProcessCloud-response>) istream)
  "Deserializes a message object of type '<ProcessCloud-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_out) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProcessCloud-response>)))
  "Returns string type for a service object of type '<ProcessCloud-response>"
  "snazzy_msgs/ProcessCloudResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProcessCloud-response)))
  "Returns string type for a service object of type 'ProcessCloud-response"
  "snazzy_msgs/ProcessCloudResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProcessCloud-response>)))
  "Returns md5sum for a message object of type '<ProcessCloud-response>"
  "62372c233a3d22e82d88cc898c7261e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProcessCloud-response)))
  "Returns md5sum for a message object of type 'ProcessCloud-response"
  "62372c233a3d22e82d88cc898c7261e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProcessCloud-response>)))
  "Returns full string definition for message of type '<ProcessCloud-response>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_out~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProcessCloud-response)))
  "Returns full string definition for message of type 'ProcessCloud-response"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_out~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProcessCloud-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_out))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProcessCloud-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ProcessCloud-response
    (cl:cons ':cloud_out (cloud_out msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ProcessCloud)))
  'ProcessCloud-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ProcessCloud)))
  'ProcessCloud-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProcessCloud)))
  "Returns string type for a service object of type '<ProcessCloud>"
  "snazzy_msgs/ProcessCloud")