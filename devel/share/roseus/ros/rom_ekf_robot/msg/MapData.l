;; Auto-generated. Do not edit!


(when (boundp 'rom_ekf_robot::MapData)
  (if (not (find-package "ROM_EKF_ROBOT"))
    (make-package "ROM_EKF_ROBOT"))
  (shadow 'MapData (find-package "ROM_EKF_ROBOT")))
(unless (find-package "ROM_EKF_ROBOT::MAPDATA")
  (make-package "ROM_EKF_ROBOT::MAPDATA"))

(in-package "ROS")
;;//! \htmlinclude MapData.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rom_ekf_robot::MapData
  :super ros::object
  :slots (_header _centers _radii ))

(defmethod rom_ekf_robot::MapData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:centers __centers) ())
    ((:radii __radii) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _centers __centers)
   (setq _radii __radii)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:centers
   (&rest __centers)
   (if (keywordp (car __centers))
       (send* _centers __centers)
     (progn
       (if __centers (setq _centers (car __centers)))
       _centers)))
  (:radii
   (&optional __radii)
   (if __radii (setq _radii __radii)) _radii)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point[] _centers
    (apply #'+ (send-all _centers :serialization-length)) 4
    ;; float64[] _radii
    (* 8    (length _radii)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point[] _centers
     (write-long (length _centers) s)
     (dolist (elem _centers)
       (send elem :serialize s)
       )
     ;; float64[] _radii
     (write-long (length _radii) s)
     (dotimes (i (length _radii))
       (sys::poke (elt _radii i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point[] _centers
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _centers (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _centers)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _radii
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _radii (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _radii i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get rom_ekf_robot::MapData :md5sum-) "ad14deec1ca094129d74da465e115243")
(setf (get rom_ekf_robot::MapData :datatype-) "rom_ekf_robot/MapData")
(setf (get rom_ekf_robot::MapData :definition-)
      "Header header
geometry_msgs/Point[] centers
float64[] radii

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :rom_ekf_robot/MapData "ad14deec1ca094129d74da465e115243")


