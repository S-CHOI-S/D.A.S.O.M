;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_workbench_msgs::WheelCommand)
  (if (not (find-package "DYNAMIXEL_WORKBENCH_MSGS"))
    (make-package "DYNAMIXEL_WORKBENCH_MSGS"))
  (shadow 'WheelCommand (find-package "DYNAMIXEL_WORKBENCH_MSGS")))
(unless (find-package "DYNAMIXEL_WORKBENCH_MSGS::WHEELCOMMAND")
  (make-package "DYNAMIXEL_WORKBENCH_MSGS::WHEELCOMMAND"))
(unless (find-package "DYNAMIXEL_WORKBENCH_MSGS::WHEELCOMMANDREQUEST")
  (make-package "DYNAMIXEL_WORKBENCH_MSGS::WHEELCOMMANDREQUEST"))
(unless (find-package "DYNAMIXEL_WORKBENCH_MSGS::WHEELCOMMANDRESPONSE")
  (make-package "DYNAMIXEL_WORKBENCH_MSGS::WHEELCOMMANDRESPONSE"))

(in-package "ROS")





(defclass dynamixel_workbench_msgs::WheelCommandRequest
  :super ros::object
  :slots (_right_vel _left_vel ))

(defmethod dynamixel_workbench_msgs::WheelCommandRequest
  (:init
   (&key
    ((:right_vel __right_vel) 0.0)
    ((:left_vel __left_vel) 0.0)
    )
   (send-super :init)
   (setq _right_vel (float __right_vel))
   (setq _left_vel (float __left_vel))
   self)
  (:right_vel
   (&optional __right_vel)
   (if __right_vel (setq _right_vel __right_vel)) _right_vel)
  (:left_vel
   (&optional __left_vel)
   (if __left_vel (setq _left_vel __left_vel)) _left_vel)
  (:serialization-length
   ()
   (+
    ;; float32 _right_vel
    4
    ;; float32 _left_vel
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _right_vel
       (sys::poke _right_vel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_vel
       (sys::poke _left_vel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _right_vel
     (setq _right_vel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_vel
     (setq _left_vel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dynamixel_workbench_msgs::WheelCommandResponse
  :super ros::object
  :slots (_result ))

(defmethod dynamixel_workbench_msgs::WheelCommandResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dynamixel_workbench_msgs::WheelCommand
  :super ros::object
  :slots ())

(setf (get dynamixel_workbench_msgs::WheelCommand :md5sum-) "a7851f940124222501142592f81f3c11")
(setf (get dynamixel_workbench_msgs::WheelCommand :datatype-) "dynamixel_workbench_msgs/WheelCommand")
(setf (get dynamixel_workbench_msgs::WheelCommand :request) dynamixel_workbench_msgs::WheelCommandRequest)
(setf (get dynamixel_workbench_msgs::WheelCommand :response) dynamixel_workbench_msgs::WheelCommandResponse)

(defmethod dynamixel_workbench_msgs::WheelCommandRequest
  (:response () (instance dynamixel_workbench_msgs::WheelCommandResponse :init)))

(setf (get dynamixel_workbench_msgs::WheelCommandRequest :md5sum-) "a7851f940124222501142592f81f3c11")
(setf (get dynamixel_workbench_msgs::WheelCommandRequest :datatype-) "dynamixel_workbench_msgs/WheelCommandRequest")
(setf (get dynamixel_workbench_msgs::WheelCommandRequest :definition-)
      "# This message is used to send velocity command to dynamixel

float32 right_vel
float32 left_vel
---
bool result

")

(setf (get dynamixel_workbench_msgs::WheelCommandResponse :md5sum-) "a7851f940124222501142592f81f3c11")
(setf (get dynamixel_workbench_msgs::WheelCommandResponse :datatype-) "dynamixel_workbench_msgs/WheelCommandResponse")
(setf (get dynamixel_workbench_msgs::WheelCommandResponse :definition-)
      "# This message is used to send velocity command to dynamixel

float32 right_vel
float32 left_vel
---
bool result

")



(provide :dynamixel_workbench_msgs/WheelCommand "a7851f940124222501142592f81f3c11")


