;; Auto-generated. Do not edit!


(when (boundp 'open_manipulator_msgs::GetKinematicsPose)
  (if (not (find-package "OPEN_MANIPULATOR_MSGS"))
    (make-package "OPEN_MANIPULATOR_MSGS"))
  (shadow 'GetKinematicsPose (find-package "OPEN_MANIPULATOR_MSGS")))
(unless (find-package "OPEN_MANIPULATOR_MSGS::GETKINEMATICSPOSE")
  (make-package "OPEN_MANIPULATOR_MSGS::GETKINEMATICSPOSE"))
(unless (find-package "OPEN_MANIPULATOR_MSGS::GETKINEMATICSPOSEREQUEST")
  (make-package "OPEN_MANIPULATOR_MSGS::GETKINEMATICSPOSEREQUEST"))
(unless (find-package "OPEN_MANIPULATOR_MSGS::GETKINEMATICSPOSERESPONSE")
  (make-package "OPEN_MANIPULATOR_MSGS::GETKINEMATICSPOSERESPONSE"))

(in-package "ROS")



(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass open_manipulator_msgs::GetKinematicsPoseRequest
  :super ros::object
  :slots ())

(defmethod open_manipulator_msgs::GetKinematicsPoseRequest
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

(defclass open_manipulator_msgs::GetKinematicsPoseResponse
  :super ros::object
  :slots (_header _kinematics_pose ))

(defmethod open_manipulator_msgs::GetKinematicsPoseResponse
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:kinematics_pose __kinematics_pose) (instance open_manipulator_msgs::KinematicsPose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _kinematics_pose __kinematics_pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
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
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; open_manipulator_msgs/KinematicsPose _kinematics_pose
    (send _kinematics_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; open_manipulator_msgs/KinematicsPose _kinematics_pose
       (send _kinematics_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; open_manipulator_msgs/KinematicsPose _kinematics_pose
     (send _kinematics_pose :deserialize buf ptr-) (incf ptr- (send _kinematics_pose :serialization-length))
   ;;
   self)
  )

(defclass open_manipulator_msgs::GetKinematicsPose
  :super ros::object
  :slots ())

(setf (get open_manipulator_msgs::GetKinematicsPose :md5sum-) "38159623face299098e8c400633c675e")
(setf (get open_manipulator_msgs::GetKinematicsPose :datatype-) "open_manipulator_msgs/GetKinematicsPose")
(setf (get open_manipulator_msgs::GetKinematicsPose :request) open_manipulator_msgs::GetKinematicsPoseRequest)
(setf (get open_manipulator_msgs::GetKinematicsPose :response) open_manipulator_msgs::GetKinematicsPoseResponse)

(defmethod open_manipulator_msgs::GetKinematicsPoseRequest
  (:response () (instance open_manipulator_msgs::GetKinematicsPoseResponse :init)))

(setf (get open_manipulator_msgs::GetKinematicsPoseRequest :md5sum-) "38159623face299098e8c400633c675e")
(setf (get open_manipulator_msgs::GetKinematicsPoseRequest :datatype-) "open_manipulator_msgs/GetKinematicsPoseRequest")
(setf (get open_manipulator_msgs::GetKinematicsPoseRequest :definition-)
      "---
Header header
KinematicsPose kinematics_pose


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
")

(setf (get open_manipulator_msgs::GetKinematicsPoseResponse :md5sum-) "38159623face299098e8c400633c675e")
(setf (get open_manipulator_msgs::GetKinematicsPoseResponse :datatype-) "open_manipulator_msgs/GetKinematicsPoseResponse")
(setf (get open_manipulator_msgs::GetKinematicsPoseResponse :definition-)
      "---
Header header
KinematicsPose kinematics_pose


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
")



(provide :open_manipulator_msgs/GetKinematicsPose "38159623face299098e8c400633c675e")


