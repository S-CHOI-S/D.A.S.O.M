;; Auto-generated. Do not edit!


(when (boundp 'open_manipulator_msgs::SetKinematicsPose)
  (if (not (find-package "OPEN_MANIPULATOR_MSGS"))
    (make-package "OPEN_MANIPULATOR_MSGS"))
  (shadow 'SetKinematicsPose (find-package "OPEN_MANIPULATOR_MSGS")))
(unless (find-package "OPEN_MANIPULATOR_MSGS::SETKINEMATICSPOSE")
  (make-package "OPEN_MANIPULATOR_MSGS::SETKINEMATICSPOSE"))
(unless (find-package "OPEN_MANIPULATOR_MSGS::SETKINEMATICSPOSEREQUEST")
  (make-package "OPEN_MANIPULATOR_MSGS::SETKINEMATICSPOSEREQUEST"))
(unless (find-package "OPEN_MANIPULATOR_MSGS::SETKINEMATICSPOSERESPONSE")
  (make-package "OPEN_MANIPULATOR_MSGS::SETKINEMATICSPOSERESPONSE"))

(in-package "ROS")





(defclass open_manipulator_msgs::SetKinematicsPoseRequest
  :super ros::object
  :slots (_kinematics_pose ))

(defmethod open_manipulator_msgs::SetKinematicsPoseRequest
  (:init
   (&key
    ((:kinematics_pose __kinematics_pose) (instance open_manipulator_msgs::KinematicsPose :init))
    )
   (send-super :init)
   (setq _kinematics_pose __kinematics_pose)
   self)
  (:kinematics_pose
   (&rest __kinematics_pose)
   (if (keywordp (car __kinematics_pose))
       (send* _kinematics_pose __kinematics_pose)
     (progn
       (if __kinematics_pose (setq _kinematics_pose (car __kinematics_pose)))
       _kinematics_pose)))
  (:serialization-length
   ()
   (+
    ;; open_manipulator_msgs/KinematicsPose _kinematics_pose
    (send _kinematics_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; open_manipulator_msgs/KinematicsPose _kinematics_pose
       (send _kinematics_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; open_manipulator_msgs/KinematicsPose _kinematics_pose
     (send _kinematics_pose :deserialize buf ptr-) (incf ptr- (send _kinematics_pose :serialization-length))
   ;;
   self)
  )

(defclass open_manipulator_msgs::SetKinematicsPoseResponse
  :super ros::object
  :slots (_isPlanned ))

(defmethod open_manipulator_msgs::SetKinematicsPoseResponse
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

(defclass open_manipulator_msgs::SetKinematicsPose
  :super ros::object
  :slots ())

(setf (get open_manipulator_msgs::SetKinematicsPose :md5sum-) "4841916847c645ef404d13cebf9595cb")
(setf (get open_manipulator_msgs::SetKinematicsPose :datatype-) "open_manipulator_msgs/SetKinematicsPose")
(setf (get open_manipulator_msgs::SetKinematicsPose :request) open_manipulator_msgs::SetKinematicsPoseRequest)
(setf (get open_manipulator_msgs::SetKinematicsPose :response) open_manipulator_msgs::SetKinematicsPoseResponse)

(defmethod open_manipulator_msgs::SetKinematicsPoseRequest
  (:response () (instance open_manipulator_msgs::SetKinematicsPoseResponse :init)))

(setf (get open_manipulator_msgs::SetKinematicsPoseRequest :md5sum-) "4841916847c645ef404d13cebf9595cb")
(setf (get open_manipulator_msgs::SetKinematicsPoseRequest :datatype-) "open_manipulator_msgs/SetKinematicsPoseRequest")
(setf (get open_manipulator_msgs::SetKinematicsPoseRequest :definition-)
      "KinematicsPose kinematics_pose

================================================================================
MSG: open_manipulator_msgs/KinematicsPose
string    	        group_name
geometry_msgs/Pose  pose
float64    max_accelerations_scaling_factor
float64    max_velocity_scaling_factor
float64    tolerance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool isPlanned

")

(setf (get open_manipulator_msgs::SetKinematicsPoseResponse :md5sum-) "4841916847c645ef404d13cebf9595cb")
(setf (get open_manipulator_msgs::SetKinematicsPoseResponse :datatype-) "open_manipulator_msgs/SetKinematicsPoseResponse")
(setf (get open_manipulator_msgs::SetKinematicsPoseResponse :definition-)
      "KinematicsPose kinematics_pose

================================================================================
MSG: open_manipulator_msgs/KinematicsPose
string    	        group_name
geometry_msgs/Pose  pose
float64    max_accelerations_scaling_factor
float64    max_velocity_scaling_factor
float64    tolerance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool isPlanned

")



(provide :open_manipulator_msgs/SetKinematicsPose "4841916847c645ef404d13cebf9595cb")


