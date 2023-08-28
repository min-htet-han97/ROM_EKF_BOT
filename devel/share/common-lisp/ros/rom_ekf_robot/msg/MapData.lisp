; Auto-generated. Do not edit!


(cl:in-package rom_ekf_robot-msg)


;//! \htmlinclude MapData.msg.html

(cl:defclass <MapData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (centers
    :reader centers
    :initarg :centers
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (radii
    :reader radii
    :initarg :radii
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass MapData (<MapData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rom_ekf_robot-msg:<MapData> is deprecated: use rom_ekf_robot-msg:MapData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MapData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rom_ekf_robot-msg:header-val is deprecated.  Use rom_ekf_robot-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'centers-val :lambda-list '(m))
(cl:defmethod centers-val ((m <MapData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rom_ekf_robot-msg:centers-val is deprecated.  Use rom_ekf_robot-msg:centers instead.")
  (centers m))

(cl:ensure-generic-function 'radii-val :lambda-list '(m))
(cl:defmethod radii-val ((m <MapData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rom_ekf_robot-msg:radii-val is deprecated.  Use rom_ekf_robot-msg:radii instead.")
  (radii m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapData>) ostream)
  "Serializes a message object of type '<MapData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'centers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'centers))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'radii))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'radii))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapData>) istream)
  "Deserializes a message object of type '<MapData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'centers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'centers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'radii) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'radii)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapData>)))
  "Returns string type for a message object of type '<MapData>"
  "rom_ekf_robot/MapData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapData)))
  "Returns string type for a message object of type 'MapData"
  "rom_ekf_robot/MapData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapData>)))
  "Returns md5sum for a message object of type '<MapData>"
  "ad14deec1ca094129d74da465e115243")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapData)))
  "Returns md5sum for a message object of type 'MapData"
  "ad14deec1ca094129d74da465e115243")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapData>)))
  "Returns full string definition for message of type '<MapData>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point[] centers~%float64[] radii~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapData)))
  "Returns full string definition for message of type 'MapData"
  (cl:format cl:nil "Header header~%geometry_msgs/Point[] centers~%float64[] radii~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'centers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'radii) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapData>))
  "Converts a ROS message object to a list"
  (cl:list 'MapData
    (cl:cons ':header (header msg))
    (cl:cons ':centers (centers msg))
    (cl:cons ':radii (radii msg))
))
