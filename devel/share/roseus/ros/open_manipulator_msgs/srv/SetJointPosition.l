;; Auto-generated. Do not edit!


(when (boundp 'open_manipulator_msgs::SetJointPosition)
  (if (not (find-package "OPEN_MANIPULATOR_MSGS"))
    (make-package "OPEN_MANIPULATOR_MSGS"))
  (shadow 'SetJointPosition (find-package "OPEN_MANIPULATOR_MSGS")))
(unless (find-package "OPEN_MANIPULATOR_MSGS::SETJOINTPOSITION")
  (make-package "OPEN_MANIPULATOR_MSGS::SETJOINTPOSITION"))
(unless (find-package "OPEN_MANIPULATOR_MSGS::SETJOINTPOSITIONREQUEST")
  (make-package "OPEN_MANIPULATOR_MSGS::SETJOINTPOSITIONREQUEST"))
(unless (find-package "OPEN_MANIPULATOR_MSGS::SETJOINTPOSITIONRESPONSE")
  (make-package "OPEN_MANIPULATOR_MSGS::SETJOINTPOSITIONRESPONSE"))

(in-package "ROS")





(defclass open_manipulator_msgs::SetJointPositionRequest
  :super ros::object
  :slots (_joint_position ))

(defmethod open_manipulator_msgs::SetJointPositionRequest
  (:init
   (&key
    ((:joint_position __joint_position) (instance open_manipulator_msgs::JointPosition :init))
    )
   (send-super :init)
   (setq _joint_position __joint_position)
   self)
  (:joint_position
   (&rest __joint_position)
   (if (keywordp (car __joint_position))
       (send* _joint_position __joint_position)
     (progn
       (if __joint_position (setq _joint_position (car __joint_position)))
       _joint_position)))
  (:serialization-length
   ()
   (+
    ;; open_manipulator_msgs/JointPosition _joint_position
    (send _joint_position :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; open_manipulator_msgs/JointPosition _joint_position
       (send _joint_position :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; open_manipulator_msgs/JointPosition _joint_position
     (send _joint_position :deserialize buf ptr-) (incf ptr- (send _joint_position :serialization-length))
   ;;
   self)
  )

(defclass open_manipulator_msgs::SetJointPositionResponse
  :super ros::object
  :slots (_isPlanned ))

(defmethod open_manipulator_msgs::SetJointPositionResponse
  (:init
   (&key
    ((:isPlanned __isPlanned) nil)
    )
   (send-super :init)
   (setq _isPlanned __isPlanned)
   self)
  (:isPlanned
   (&optional __isPlanned)
   (if __isPlanned (setq _isPlanned __isPlanned)) _isPlanned)
  (:serialization-length
   ()
   (+
    ;; bool _isPlanned
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isPlanned
       (if _isPlanned (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isPlanned
     (setq _isPlanned (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass open_manipulator_msgs::SetJointPosition
  :super ros::object
  :slots ())

(setf (get open_manipulator_msgs::SetJointPosition :md5sum-) "51816c1a47e918af5b30ea571f8202f7")
(setf (get open_manipulator_msgs::SetJointPosition :datatype-) "open_manipulator_msgs/SetJointPosition")
(setf (get open_manipulator_msgs::SetJointPosition :request) open_manipulator_msgs::SetJointPositionRequest)
(setf (get open_manipulator_msgs::SetJointPosition :response) open_manipulator_msgs::SetJointPositionResponse)

(defmethod open_manipulator_msgs::SetJointPositionRequest
  (:response () (instance open_manipulator_msgs::SetJointPositionResponse :init)))

(setf (get open_manipulator_msgs::SetJointPositionRequest :md5sum-) "51816c1a47e918af5b30ea571f8202f7")
(setf (get open_manipulator_msgs::SetJointPositionRequest :datatype-) "open_manipulator_msgs/SetJointPositionRequest")
(setf (get open_manipulator_msgs::SetJointPositionRequest :definition-)
      "JointPosition joint_position

================================================================================
MSG: open_manipulator_msgs/JointPosition
string[]   joint_name
float64[]  position
float64    max_accelerations_scaling_factor
float64    max_velocity_scaling_factor
---
bool isPlanned

")

(setf (get open_manipulator_msgs::SetJointPositionResponse :md5sum-) "51816c1a47e918af5b30ea571f8202f7")
(setf (get open_manipulator_msgs::SetJointPositionResponse :datatype-) "open_manipulator_msgs/SetJointPositionResponse")
(setf (get open_manipulator_msgs::SetJointPositionResponse :definition-)
      "JointPosition joint_position

================================================================================
MSG: open_manipulator_msgs/JointPosition
string[]   joint_name
float64[]  position
float64    max_accelerations_scaling_factor
float64    max_velocity_scaling_factor
---
bool isPlanned

")



(provide :open_manipulator_msgs/SetJointPosition "51816c1a47e918af5b30ea571f8202f7")


