; Auto-generated. Do not edit!


(cl:in-package open_manipulator_msgs-srv)


;//! \htmlinclude GetKinematicsPose-request.msg.html

(cl:defclass <GetKinematicsPose-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetKinematicsPose-request (<GetKinematicsPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetKinematicsPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetKinematicsPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name open_manipulator_msgs-srv:<GetKinematicsPose-request> is deprecated: use open_manipulator_msgs-srv:GetKinematicsPose-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetKinematicsPose-request>) ostream)
  "Serializes a message object of type '<GetKinematicsPose-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetKinematicsPose-request>) istream)
  "Deserializes a message object of type '<GetKinematicsPose-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetKinematicsPose-request>)))
  "Returns string type for a service object of type '<GetKinematicsPose-request>"
  "open_manipulator_msgs/GetKinematicsPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKinematicsPose-request)))
  "Returns string type for a service object of type 'GetKinematicsPose-request"
  "open_manipulator_msgs/GetKinematicsPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetKinematicsPose-request>)))
  "Returns md5sum for a message object of type '<GetKinematicsPose-request>"
  "38159623face299098e8c400633c675e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetKinematicsPose-request)))
  "Returns md5sum for a message object of type 'GetKinematicsPose-request"
  "38159623face299098e8c400633c675e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetKinematicsPose-request>)))
  "Returns full string definition for message of type '<GetKinematicsPose-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetKinematicsPose-request)))
  "Returns full string definition for message of type 'GetKinematicsPose-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetKinematicsPose-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetKinematicsPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetKinematicsPose-request
))
;//! \htmlinclude GetKinematicsPose-response.msg.html

(cl:defclass <GetKinematicsPose-response> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (kinematics_pose
    :reader kinematics_pose
    :initarg :kinematics_pose
    :type open_manipulator_msgs-msg:KinematicsPose
    :initform (cl:make-instance 'open_manipulator_msgs-msg:KinematicsPose)))
)

(cl:defclass GetKinematicsPose-response (<GetKinematicsPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetKinematicsPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetKinematicsPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name open_manipulator_msgs-srv:<GetKinematicsPose-response> is deprecated: use open_manipulator_msgs-srv:GetKinematicsPose-response instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GetKinematicsPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_manipulator_msgs-srv:header-val is deprecated.  Use open_manipulator_msgs-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'kinematics_pose-val :lambda-list '(m))
(cl:defmethod kinematics_pose-val ((m <GetKinematicsPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_manipulator_msgs-srv:kinematics_pose-val is deprecated.  Use open_manipulator_msgs-srv:kinematics_pose instead.")
  (kinematics_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetKinematicsPose-response>) ostream)
  "Serializes a message object of type '<GetKinematicsPose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'kinematics_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetKinematicsPose-response>) istream)
  "Deserializes a message object of type '<GetKinematicsPose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'kinematics_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetKinematicsPose-response>)))
  "Returns string type for a service object of type '<GetKinematicsPose-response>"
  "open_manipulator_msgs/GetKinematicsPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKinematicsPose-response)))
  "Returns string type for a service object of type 'GetKinematicsPose-response"
  "open_manipulator_msgs/GetKinematicsPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetKinematicsPose-response>)))
  "Returns md5sum for a message object of type '<GetKinematicsPose-response>"
  "38159623face299098e8c400633c675e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetKinematicsPose-response)))
  "Returns md5sum for a message object of type 'GetKinematicsPose-response"
  "38159623face299098e8c400633c675e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetKinematicsPose-response>)))
  "Returns full string definition for message of type '<GetKinematicsPose-response>"
  (cl:format cl:nil "Header header~%KinematicsPose kinematics_pose~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: open_manipulator_msgs/KinematicsPose~%string    	        group_name~%geometry_msgs/Pose  pose~%float64    max_accelerations_scaling_factor~%float64    max_velocity_scaling_factor~%float64    tolerance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetKinematicsPose-response)))
  "Returns full string definition for message of type 'GetKinematicsPose-response"
  (cl:format cl:nil "Header header~%KinematicsPose kinematics_pose~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: open_manipulator_msgs/KinematicsPose~%string    	        group_name~%geometry_msgs/Pose  pose~%float64    max_accelerations_scaling_factor~%float64    max_velocity_scaling_factor~%float64    tolerance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetKinematicsPose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'kinematics_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetKinematicsPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetKinematicsPose-response
    (cl:cons ':header (header msg))
    (cl:cons ':kinematics_pose (kinematics_pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetKinematicsPose)))
  'GetKinematicsPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetKinematicsPose)))
  'GetKinematicsPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKinematicsPose)))
  "Returns string type for a service object of type '<GetKinematicsPose>"
  "open_manipulator_msgs/GetKinematicsPose")