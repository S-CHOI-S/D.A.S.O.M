;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_workbench_msgs::EECommand)
  (if (not (find-package "DYNAMIXEL_WORKBENCH_MSGS"))
    (make-package "DYNAMIXEL_WORKBENCH_MSGS"))
  (shadow 'EECommand (find-package "DYNAMIXEL_WORKBENCH_MSGS")))
(unless (find-package "DYNAMIXEL_WORKBENCH_MSGS::EECOMMAND")
  (make-package "DYNAMIXEL_WORKBENCH_MSGS::EECOMMAND"))
(unless (find-package "DYNAMIXEL_WORKBENCH_MSGS::EECOMMANDREQUEST")
  (make-package "DYNAMIXEL_WORKBENCH_MSGS::EECOMMANDREQUEST"))
(unless (find-package "DYNAMIXEL_WORKBENCH_MSGS::EECOMMANDRESPONSE")
  (make-package "DYNAMIXEL_WORKBENCH_MSGS::EECOMMANDRESPONSE"))

(in-package "ROS")





(defclass dynamixel_workbench_msgs::EECommandRequest
  :super ros::object
  :slots (_X _Y ))

(defmethod dynamixel_workbench_msgs::EECommandRequest
  (:init
   (&key
    ((:X __X) 0.0)
    ((:Y __Y) 0.0)
    )
   (send-super :init)
   (setq _X (float __X))
   (setq _Y (float __Y))
   self)
  (:X
   (&optional __X)
   (if __X (setq _X __X)) _X)
  (:Y
   (&optional __Y)
   (if __Y (setq _Y __Y)) _Y)
  (:serialization-length
   ()
   (+
    ;; float64 _X
    8
    ;; float64 _Y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _X
       (sys::poke _X (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Y
       (sys::poke _Y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _X
     (setq _X (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Y
     (setq _Y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass dynamixel_workbench_msgs::EECommandResponse
  :super ros::object
  :slots ())

(defmethod dynamixel_workbench_msgs::EECommandResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass dynamixel_workbench_msgs::EECommand
  :super ros::object
  :slots ())

(setf (get dynamixel_workbench_msgs::EECommand :md5sum-) "e7f17a0f3bbae3e0b8d5b7c65426d275")
(setf (get dynamixel_workbench_msgs::EECommand :datatype-) "dynamixel_workbench_msgs/EECommand")
(setf (get dynamixel_workbench_msgs::EECommand :request) dynamixel_workbench_msgs::EECommandRequest)
(setf (get dynamixel_workbench_msgs::EECommand :response) dynamixel_workbench_msgs::EECommandResponse)

(defmethod dynamixel_workbench_msgs::EECommandRequest
  (:response () (instance dynamixel_workbench_msgs::EECommandResponse :init)))

(setf (get dynamixel_workbench_msgs::EECommandRequest :md5sum-) "e7f17a0f3bbae3e0b8d5b7c65426d275")
(setf (get dynamixel_workbench_msgs::EECommandRequest :datatype-) "dynamixel_workbench_msgs/EECommandRequest")
(setf (get dynamixel_workbench_msgs::EECommandRequest :definition-)
      "# This message is used to send End-Effector command to DASOM

float64 X
float64 Y
---


")

(setf (get dynamixel_workbench_msgs::EECommandResponse :md5sum-) "e7f17a0f3bbae3e0b8d5b7c65426d275")
(setf (get dynamixel_workbench_msgs::EECommandResponse :datatype-) "dynamixel_workbench_msgs/EECommandResponse")
(setf (get dynamixel_workbench_msgs::EECommandResponse :definition-)
      "# This message is used to send End-Effector command to DASOM

float64 X
float64 Y
---


")



(provide :dynamixel_workbench_msgs/EECommand "e7f17a0f3bbae3e0b8d5b7c65426d275")


