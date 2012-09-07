; Auto-generated. Do not edit!


(cl:in-package snazzy_msgs-msg)


;//! \htmlinclude CropBox.msg.html

(cl:defclass <CropBox> (roslisp-msg-protocol:ros-message)
  ((mins
    :reader mins
    :initarg :mins
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (maxes
    :reader maxes
    :initarg :maxes
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass CropBox (<CropBox>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CropBox>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CropBox)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name snazzy_msgs-msg:<CropBox> is deprecated: use snazzy_msgs-msg:CropBox instead.")))

(cl:ensure-generic-function 'mins-val :lambda-list '(m))
(cl:defmethod mins-val ((m <CropBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snazzy_msgs-msg:mins-val is deprecated.  Use snazzy_msgs-msg:mins instead.")
  (mins m))

(cl:ensure-generic-function 'maxes-val :lambda-list '(m))
(cl:defmethod maxes-val ((m <CropBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snazzy_msgs-msg:maxes-val is deprecated.  Use snazzy_msgs-msg:maxes instead.")
  (maxes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CropBox>) ostream)
  "Serializes a message object of type '<CropBox>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mins))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'mins))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'maxes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'maxes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CropBox>) istream)
  "Deserializes a message object of type '<CropBox>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mins) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mins)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'maxes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'maxes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CropBox>)))
  "Returns string type for a message object of type '<CropBox>"
  "snazzy_msgs/CropBox")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CropBox)))
  "Returns string type for a message object of type 'CropBox"
  "snazzy_msgs/CropBox")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CropBox>)))
  "Returns md5sum for a message object of type '<CropBox>"
  "020b69f3a9bb46f8c0f0660ed67bdc6a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CropBox)))
  "Returns md5sum for a message object of type 'CropBox"
  "020b69f3a9bb46f8c0f0660ed67bdc6a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CropBox>)))
  "Returns full string definition for message of type '<CropBox>"
  (cl:format cl:nil "float32[] mins~%float32[] maxes~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CropBox)))
  "Returns full string definition for message of type 'CropBox"
  (cl:format cl:nil "float32[] mins~%float32[] maxes~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CropBox>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mins) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'maxes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CropBox>))
  "Converts a ROS message object to a list"
  (cl:list 'CropBox
    (cl:cons ':mins (mins msg))
    (cl:cons ':maxes (maxes msg))
))
