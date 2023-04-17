;; Auto-generated. Do not edit!


(when (boundp 'test::Torqbian)
  (if (not (find-package "TEST"))
    (make-package "TEST"))
  (shadow 'Torqbian (find-package "TEST")))
(unless (find-package "TEST::TORQBIAN")
  (make-package "TEST::TORQBIAN"))

(in-package "ROS")
;;//! \htmlinclude Torqbian.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass test::Torqbian
  :super ros::object
  :slots (_header _X _Y _cmd_position ))

(defmethod test::Torqbian
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:X __X) 0.0)
    ((:Y __Y) 0.0)
    ((:cmd_position __cmd_position) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _X (float __X))
   (setq _Y (float __Y))
   (setq _cmd_position __cmd_position)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:X
   (&optional __X)
   (if __X (setq _X __X)) _X)
  (:Y
   (&optional __Y)
   (if __Y (setq _Y __Y)) _Y)
  (:cmd_position
   (&optional __cmd_position)
   (if __cmd_position (setq _cmd_position __cmd_position)) _cmd_position)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _X
    8
    ;; float64 _Y
    8
    ;; float64[] _cmd_position
    (* 8    (length _cmd_position)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _X
       (sys::poke _X (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Y
       (sys::poke _Y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _cmd_position
     (write-long (length _cmd_position) s)
     (dotimes (i (length _cmd_position))
       (sys::poke (elt _cmd_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _X
     (setq _X (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Y
     (setq _Y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _cmd_position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cmd_position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cmd_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get test::Torqbian :md5sum-) "06d2af97fff100f9314c1ca3f4dfbf26")
(setf (get test::Torqbian :datatype-) "test/Torqbian")
(setf (get test::Torqbian :definition-)
      "Header header

float64 X
float64 Y
float64[] cmd_position

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



(provide :test/Torqbian "06d2af97fff100f9314c1ca3f4dfbf26")


