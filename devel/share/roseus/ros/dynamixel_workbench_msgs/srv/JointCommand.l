;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_workbench_msgs::JointCommand)
  (if (not (find-package "DYNAMIXEL_WORKBENCH_MSGS"))
    (make-package "DYNAMIXEL_WORKBENCH_MSGS"))
  (shadow 'JointCommand (find-package "DYNAMIXEL_WORKBENCH_MSGS")))
(unless (find-package "DYNAMIXEL_WORKBENCH_MSGS::JOINTCOMMAND")
  (make-package "DYNAMIXEL_WORKBENCH_MSGS::JOINTCOMMAND"))
(unless (find-package "DYNAMIXEL_WORKBENCH_MSGS::JOINTCOMMANDREQUEST")
  (make-package "DYNAMIXEL_WORKBENCH_MSGS::JOINTCOMMANDREQUEST"))
(unless (find-package "DYNAMIXEL_WORKBENCH_MSGS::JOINTCOMMANDRESPONSE")
  (make-package "DYNAMIXEL_WORKBENCH_MSGS::JOINTCOMMANDRESPONSE"))

(in-package "ROS")





(defclass dynamixel_workbench_msgs::JointCommandRequest
  :super ros::object
  :slots (_unit _id _goal_position _goal_position_2 ))

(defmethod dynamixel_workbench_msgs::JointCommandRequest
  (:init
   (&key
    ((:unit __unit) "")
    ((:id __id) 0)
    ((:goal_position __goal_position) 0.0)
    ((:goal_position_2 __goal_position_2) 0.0)
    )
   (send-super :init)
   (setq _unit (string __unit))
   (setq _id (round __id))
   (setq _goal_position (float __goal_position))
   (setq _goal_position_2 (float __goal_position_2))
   self)
  (:unit
   (&optional __unit)
   (if __unit (setq _unit __unit)) _unit)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:goal_position
   (&optional __goal_position)
   (if __goal_position (setq _goal_position __goal_position)) _goal_position)
  (:goal_position_2
   (&optional __goal_position_2)
   (if __goal_position_2 (setq _goal_position_2 __goal_position_2)) _goal_position_2)
  (:serialization-length
   ()
   (+
    ;; string _unit
    4 (length _unit)
    ;; uint8 _id
    1
    ;; float32 _goal_position
    4
    ;; float32 _goal_position_2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _unit
       (write-long (length _unit) s) (princ _unit s)
     ;; uint8 _id
       (write-byte _id s)
     ;; float32 _goal_position
       (sys::poke _goal_position (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _goal_position_2
       (sys::poke _goal_position_2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _unit
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _unit (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _goal_position
     (setq _goal_position (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _goal_position_2
     (setq _goal_position_2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dynamixel_workbench_msgs::JointCommandResponse
  :super ros::object
  :slots (_result ))

(defmethod dynamixel_workbench_msgs::JointCommandResponse
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

(defclass dynamixel_workbench_msgs::JointCommand
  :super ros::object
  :slots ())

(setf (get dynamixel_workbench_msgs::JointCommand :md5sum-) "0f59c59a18a44b029e233012bc29d67d")
(setf (get dynamixel_workbench_msgs::JointCommand :datatype-) "dynamixel_workbench_msgs/JointCommand")
(setf (get dynamixel_workbench_msgs::JointCommand :request) dynamixel_workbench_msgs::JointCommandRequest)
(setf (get dynamixel_workbench_msgs::JointCommand :response) dynamixel_workbench_msgs::JointCommandResponse)

(defmethod dynamixel_workbench_msgs::JointCommandRequest
  (:response () (instance dynamixel_workbench_msgs::JointCommandResponse :init)))

(setf (get dynamixel_workbench_msgs::JointCommandRequest :md5sum-) "0f59c59a18a44b029e233012bc29d67d")
(setf (get dynamixel_workbench_msgs::JointCommandRequest :datatype-) "dynamixel_workbench_msgs/JointCommandRequest")
(setf (get dynamixel_workbench_msgs::JointCommandRequest :definition-)
      "# This message is used to send position command to dynamixel

string unit
uint8 id
float32 goal_position
float32 goal_position_2
---
bool result

")

(setf (get dynamixel_workbench_msgs::JointCommandResponse :md5sum-) "0f59c59a18a44b029e233012bc29d67d")
(setf (get dynamixel_workbench_msgs::JointCommandResponse :datatype-) "dynamixel_workbench_msgs/JointCommandResponse")
(setf (get dynamixel_workbench_msgs::JointCommandResponse :definition-)
      "# This message is used to send position command to dynamixel

string unit
uint8 id
float32 goal_position
float32 goal_position_2
---
bool result

")



(provide :dynamixel_workbench_msgs/JointCommand "0f59c59a18a44b029e233012bc29d67d")


