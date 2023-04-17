; Auto-generated. Do not edit!


(cl:in-package open_manipulator_msgs-srv)


;//! \htmlinclude SetJointPosition-request.msg.html

(cl:defclass <SetJointPosition-request> (roslisp-msg-protocol:ros-message)
  ((joint_position
    :reader joint_position
    :initarg :joint_position
    :type open_manipulator_msgs-msg:JointPosition
    :initform (cl:make-instance 'open_manipulator_msgs-msg:JointPosition)))
)

(cl:defclass SetJointPosition-request (<SetJointPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name open_manipulator_msgs-srv:<SetJointPosition-request> is deprecated: use open_manipulator_msgs-srv:SetJointPosition-request instead.")))

(cl:ensure-generic-function 'joint_position-val :lambda-list '(m))
(cl:defmethod joint_position-val ((m <SetJointPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_manipulator_msgs-srv:joint_position-val is deprecated.  Use open_manipulator_msgs-srv:joint_position instead.")
  (joint_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointPosition-request>) ostream)
  "Serializes a message object of type '<SetJointPosition-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointPosition-request>) istream)
  "Deserializes a message object of type '<SetJointPosition-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointPosition-request>)))
  "Returns string type for a service object of type '<SetJointPosition-request>"
  "open_manipulator_msgs/SetJointPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointPosition-request)))
  "Returns string type for a service object of type 'SetJointPosition-request"
  "open_manipulator_msgs/SetJointPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointPosition-request>)))
  "Returns md5sum for a message object of type '<SetJointPosition-request>"
  "51816c1a47e918af5b30ea571f8202f7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointPosition-request)))
  "Returns md5sum for a message object of type 'SetJointPosition-request"
  "51816c1a47e918af5b30ea571f8202f7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointPosition-request>)))
  "Returns full string definition for message of type '<SetJointPosition-request>"
  (cl:format cl:nil "JointPosition joint_position~%~%================================================================================~%MSG: open_manipulator_msgs/JointPosition~%string[]   joint_name~%float64[]  position~%float64    max_accelerations_scaling_factor~%float64    max_velocity_scaling_factor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointPosition-request)))
  "Returns full string definition for message of type 'SetJointPosition-request"
  (cl:format cl:nil "JointPosition joint_position~%~%================================================================================~%MSG: open_manipulator_msgs/JointPosition~%string[]   joint_name~%float64[]  position~%float64    max_accelerations_scaling_factor~%float64    max_velocity_scaling_factor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointPosition-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointPosition-request
    (cl:cons ':joint_position (joint_position msg))
))
;//! \htmlinclude SetJointPosition-response.msg.html

(cl:defclass <SetJointPosition-response> (roslisp-msg-protocol:ros-message)
  ((isPlanned
    :reader isPlanned
    :initarg :isPlanned
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetJointPosition-response (<SetJointPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name open_manipulator_msgs-srv:<SetJointPosition-response> is deprecated: use open_manipulator_msgs-srv:SetJointPosition-response instead.")))

(cl:ensure-generic-function 'isPlanned-val :lambda-list '(m))
(cl:defmethod isPlanned-val ((m <SetJointPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_manipulator_msgs-srv:isPlanned-val is deprecated.  Use open_manipulator_msgs-srv:isPlanned instead.")
  (isPlanned m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointPosition-response>) ostream)
  "Serializes a message object of type '<SetJointPosition-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isPlanned) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointPosition-response>) istream)
  "Deserializes a message object of type '<SetJointPosition-response>"
    (cl:setf (cl:slot-value msg 'isPlanned) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointPosition-response>)))
  "Returns string type for a service object of type '<SetJointPosition-response>"
  "open_manipulator_msgs/SetJointPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointPosition-response)))
  "Returns string type for a service object of type 'SetJointPosition-response"
  "open_manipulator_msgs/SetJointPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointPosition-response>)))
  "Returns md5sum for a message object of type '<SetJointPosition-response>"
  "51816c1a47e918af5b30ea571f8202f7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointPosition-response)))
  "Returns md5sum for a message object of type 'SetJointPosition-response"
  "51816c1a47e918af5b30ea571f8202f7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointPosition-response>)))
  "Returns full string definition for message of type '<SetJointPosition-response>"
  (cl:format cl:nil "bool isPlanned~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointPosition-response)))
  "Returns full string definition for message of type 'SetJointPosition-response"
  (cl:format cl:nil "bool isPlanned~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointPosition-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointPosition-response
    (cl:cons ':isPlanned (isPlanned msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetJointPosition)))
  'SetJointPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetJointPosition)))
  'SetJointPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointPosition)))
  "Returns string type for a service object of type '<SetJointPosition>"
  "open_manipulator_msgs/SetJointPosition")