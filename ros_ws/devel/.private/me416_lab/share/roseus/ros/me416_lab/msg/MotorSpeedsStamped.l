;; Auto-generated. Do not edit!


(when (boundp 'me416_lab::MotorSpeedsStamped)
  (if (not (find-package "ME416_LAB"))
    (make-package "ME416_LAB"))
  (shadow 'MotorSpeedsStamped (find-package "ME416_LAB")))
(unless (find-package "ME416_LAB::MOTORSPEEDSSTAMPED")
  (make-package "ME416_LAB::MOTORSPEEDSSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude MotorSpeedsStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass me416_lab::MotorSpeedsStamped
  :super ros::object
  :slots (_header _left _right ))

(defmethod me416_lab::MotorSpeedsStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left __left) 0.0)
    ((:right __right) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _left (float __left))
   (setq _right (float __right))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _left
    8
    ;; float64 _right
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _left
       (sys::poke _left (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right
       (sys::poke _right (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _left
     (setq _left (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right
     (setq _right (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get me416_lab::MotorSpeedsStamped :md5sum-) "8f32685125452f5bdf68130369af5296")
(setf (get me416_lab::MotorSpeedsStamped :datatype-) "me416_lab/MotorSpeedsStamped")
(setf (get me416_lab::MotorSpeedsStamped :definition-)
      "Header header
float64 left
float64 right

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

")



(provide :me416_lab/MotorSpeedsStamped "8f32685125452f5bdf68130369af5296")


