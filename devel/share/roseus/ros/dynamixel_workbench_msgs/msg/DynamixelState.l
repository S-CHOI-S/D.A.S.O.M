;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_workbench_msgs::DynamixelState)
  (if (not (find-package "DYNAMIXEL_WORKBENCH_MSGS"))
    (make-package "DYNAMIXEL_WORKBENCH_MSGS"))
  (shadow 'DynamixelState (find-package "DYNAMIXEL_WORKBENCH_MSGS")))
(unless (find-package "DYNAMIXEL_WORKBENCH_MSGS::DYNAMIXELSTATE")
  (make-package "DYNAMIXEL_WORKBENCH_MSGS::DYNAMIXELSTATE"))

(in-package "ROS")
;;//! \htmlinclude DynamixelState.msg.html


(defclass dynamixel_workbench_msgs::DynamixelState
  :super ros::object
  :slots (_model_name _id _torque_enable _goal_current _goal_velocity _goal_position _present_current _present_velocity _present_position _moving ))

(defmethod dynamixel_workbench_msgs::DynamixelState
  (:init
   (&key
    ((:model_name __model_name) "")
    ((:id __id) 0)
    ((:torque_enable __torque_enable) 0)
    ((:goal_current __goal_current) 0)
    ((:goal_velocity __goal_velocity) 0)
    ((:goal_position __goal_position) 0)
    ((:present_current __present_current) 0)
    ((:present_velocity __present_velocity) 0)
    ((:present_position __present_position) 0)
    ((:moving __moving) 0)
    )
   (send-super :init)
   (setq _model_name (string __model_name))
   (setq _id (round __id))
   (setq _torque_enable (round __torque_enable))
   (setq _goal_current (round __goal_current))
   (setq _goal_velocity (round __goal_velocity))
   (setq _goal_position (round __goal_position))
   (setq _present_current (round __present_current))
   (setq _present_velocity (round __present_velocity))
   (setq _present_position (round __present_position))
   (setq _moving (round __moving))
   self)
  (:model_name
   (&optional __model_name)
   (if __model_name (setq _model_name __model_name)) _model_name)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:torque_enable
   (&optional __torque_enable)
   (if __torque_enable (setq _torque_enable __torque_enable)) _torque_enable)
  (:goal_current
   (&optional __goal_current)
   (if __goal_current (setq _goal_current __goal_current)) _goal_current)
  (:goal_velocity
   (&optional __goal_velocity)
   (if __goal_velocity (setq _goal_velocity __goal_velocity)) _goal_velocity)
  (:goal_position
   (&optional __goal_position)
   (if __goal_position (setq _goal_position __goal_position)) _goal_position)
  (:present_current
   (&optional __present_current)
   (if __present_current (setq _present_current __present_current)) _present_current)
  (:present_velocity
   (&optional __present_velocity)
   (if __present_velocity (setq _present_velocity __present_velocity)) _present_velocity)
  (:present_position
   (&optional __present_position)
   (if __present_position (setq _present_position __present_position)) _present_position)
  (:moving
   (&optional __moving)
   (if __moving (setq _moving __moving)) _moving)
  (:serialization-length
   ()
   (+
    ;; string _model_name
    4 (length _model_name)
    ;; uint8 _id
    1
    ;; uint8 _torque_enable
    1
    ;; int16 _goal_current
    2
    ;; int32 _goal_velocity
    4
    ;; int32 _goal_position
    4
    ;; int16 _present_current
    2
    ;; int32 _present_velocity
    4
    ;; int32 _present_position
    4
    ;; uint8 _moving
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _model_name
       (write-long (length _model_name) s) (princ _model_name s)
     ;; uint8 _id
       (write-byte _id s)
     ;; uint8 _torque_enable
       (write-byte _torque_enable s)
     ;; int16 _goal_current
       (write-word _goal_current s)
     ;; int32 _goal_velocity
       (write-long _goal_velocity s)
     ;; int32 _goal_position
       (write-long _goal_position s)
     ;; int16 _present_current
       (write-word _present_current s)
     ;; int32 _present_velocity
       (write-long _present_velocity s)
     ;; int32 _present_position
       (write-long _present_position s)
     ;; uint8 _moving
       (write-byte _moving s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _model_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _torque_enable
     (setq _torque_enable (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int16 _goal_current
     (setq _goal_current (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int32 _goal_velocity
     (setq _goal_velocity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _goal_position
     (setq _goal_position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16 _present_current
     (setq _present_current (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int32 _present_velocity
     (setq _present_velocity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _present_position
     (setq _present_position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _moving
     (setq _moving (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get dynamixel_workbench_msgs::DynamixelState :md5sum-) "75ef89ec4862f78146303f5792bac9b7")
(setf (get dynamixel_workbench_msgs::DynamixelState :datatype-) "dynamixel_workbench_msgs/DynamixelState")
(setf (get dynamixel_workbench_msgs::DynamixelState :definition-)
      "# This message includes basic data of dynamixel

string model_name
uint8  id
uint8  torque_enable

int16  goal_current
int32  goal_velocity
int32 goal_position

int16  present_current
int32  present_velocity
int32 present_position

uint8 moving


")



(provide :dynamixel_workbench_msgs/DynamixelState "75ef89ec4862f78146303f5792bac9b7")


