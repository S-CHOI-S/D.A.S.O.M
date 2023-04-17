; Auto-generated. Do not edit!


(cl:in-package open_manipulator_msgs-srv)


;//! \htmlinclude SetKinematicsPose-request.msg.html

(cl:defclass <SetKinematicsPose-request> (roslisp-msg-protocol:ros-message)
  ((kinematics_pose
    :reader kinematics_pose
    :initarg :kinematics_pose
    :type open_manipulator_msgs-msg:KinematicsPose
    :initform (cl:make-instance 'open_manipulator_msgs-msg:KinematicsPose)))
)

(cl:defclass SetKinematicsPose-request (<SetKinematicsPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetKinematicsPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetKinematicsPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name open_manipulator_msgs-srv:<SetKinematicsPose-request> is deprecated: use open_manipulator_msgs-srv:SetKinematicsPose-request instead.")))

(cl:ensure-generic-function 'kinematics_pose-val :lambda-list '(m))
(cl:defmethod kinematics_pose-val ((m <SetKinematicsPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_manipulator_msgs-srv:kinematics_pose-val is deprecated.  Use open_manipulator_msgs-srv:kinematics_pose instead.")
  (kinematics_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetKinematicsPose-request>) ostream)
  "Serializes a message object of type '<SetKinematicsPose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'kinematics_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetKinematicsPose-request>) istream)
  "Deserializes a message object of type '<SetKinematicsPose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'kinematics_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetKinematicsPose-request>)))
  "Returns string type for a service object of type '<SetKinematicsPose-request>"
  "open_manipulator_msgs/SetKinematicsPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetKinematicsPose-request)))
  "Returns string type for a service object of type 'SetKinematicsPose-request"
  "open_manipulator_msgs/SetKinematicsPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetKinematicsPose-request>)))
  "Returns md5sum for a message object of type '<SetKinematicsPose-request>"
  "4841916847c645ef404d13cebf9595cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetKinematicsPose-request)))
  "Returns md5sum for a message object of type 'SetKinematicsPose-request"
  "4841916847c645ef404d13cebf9595cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetKinematicsPose-request>)))
  "Returns full string definition for message of type '<SetKinematicsPose-request>"
  (cl:format cl:nil "KinematicsPose kinematics_pose~%~%================================================================================~%MSG: open_manipulator_msgs/KinematicsPose~%string    	        group_name~%geometry_msgs/Pose  pose~%float64    max_accelerations_scaling_factor~%float64    max_velocity_scaling_factor~%float64    tolerance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetKinematicsPose-request)))
  "Returns full string definition for message of type 'SetKinematicsPose-request"
  (cl:format cl:nil "KinematicsPose kinematics_pose~%~%================================================================================~%MSG: open_manipulator_msgs/KinematicsPose~%string    	        group_name~%geometry_msgs/Pose  pose~%float64    max_accelerations_scaling_factor~%float64    max_velocity_scaling_factor~%float64    tolerance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetKinematicsPose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'kinematics_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetKinematicsPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetKinematicsPose-request
    (cl:cons ':kinematics_pose (kinematics_pose msg))
))
;//! \htmlinclude SetKinematicsPose-response.msg.html

(cl:defclass <SetKinematicsPose-response> (roslisp-msg-protocol:ros-message)
  ((isPlanned
    :reader isPlanned
    :initarg :isPlanned
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetKinematicsPose-response (<SetKinematicsPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetKinematicsPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetKinematicsPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name open_manipulator_msgs-srv:<SetKinematicsPose-response> is deprecated: use open_manipulator_msgs-srv:SetKinematicsPose-response instead.")))

(cl:ensure-generic-function 'isPlanned-val :lambda-list '(m))
(cl:defmethod isPlanned-val ((m <SetKinematicsPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_manipulator_msgs-srv:isPlanned-val is deprecated.  Use open_manipulator_msgs-srv:isPlanned instead.")
  (isPlanned m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetKinematicsPose-response>) ostream)
  "Serializes a message object of type '<SetKinematicsPose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isPlanned) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetKinematicsPose-response>) istream)
  "Deserializes a message object of type '<SetKinematicsPose-response>"
    (cl:setf (cl:slot-value msg 'isPlanned) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetKinematicsPose-response>)))
  "Returns string type for a service object of type '<SetKinematicsPose-response>"
  "open_manipulator_msgs/SetKinematicsPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetKinematicsPose-response)))
  "Returns string type for a service object of type 'SetKinematicsPose-response"
  "open_manipulator_msgs/SetKinematicsPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetKinematicsPose-response>)))
  "Returns md5sum for a message object of type '<SetKinematicsPose-response>"
  "4841916847c645ef404d13cebf9595cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetKinematicsPose-response)))
  "Returns md5sum for a message object of type 'SetKinematicsPose-response"
  "4841916847c645ef404d13cebf9595cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetKinematicsPose-response>)))
  "Returns full string definition for message of type '<SetKinematicsPose-response>"
  (cl:format cl:nil "bool isPlanned~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetKinematicsPose-response)))
  "Returns full string definition for message of type 'SetKinematicsPose-response"
  (cl:format cl:nil "bool isPlanned~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetKinematicsPose-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetKinematicsPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetKinematicsPose-response
    (cl:cons ':isPlanned (isPlanned msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetKinematicsPose)))
  'SetKinematicsPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetKinematicsPose)))
  'SetKinematicsPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetKinematicsPose)))
  "Returns string type for a service object of type '<SetKinematicsPose>"
  "open_manipulator_msgs/SetKinematicsPose")