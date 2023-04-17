; Auto-generated. Do not edit!


(cl:in-package open_manipulator_msgs-srv)


;//! \htmlinclude GetJointPosition-request.msg.html

(cl:defclass <GetJointPosition-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetJointPosition-request (<GetJointPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetJointPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetJointPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name open_manipulator_msgs-srv:<GetJointPosition-request> is deprecated: use open_manipulator_msgs-srv:GetJointPosition-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetJointPosition-request>) ostream)
  "Serializes a message object of type '<GetJointPosition-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetJointPosition-request>) istream)
  "Deserializes a message object of type '<GetJointPosition-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetJointPosition-request>)))
  "Returns string type for a service object of type '<GetJointPosition-request>"
  "open_manipulator_msgs/GetJointPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointPosition-request)))
  "Returns string type for a service object of type 'GetJointPosition-request"
  "open_manipulator_msgs/GetJointPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetJointPosition-request>)))
  "Returns md5sum for a message object of type '<GetJointPosition-request>"
  "e1f1ee99b5e77308297dc4eeedd305d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetJointPosition-request)))
  "Returns md5sum for a message object of type 'GetJointPosition-request"
  "e1f1ee99b5e77308297dc4eeedd305d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetJointPosition-request>)))
  "Returns full string definition for message of type '<GetJointPosition-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetJointPosition-request)))
  "Returns full string definition for message of type 'GetJointPosition-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetJointPosition-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetJointPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetJointPosition-request
))
;//! \htmlinclude GetJointPosition-response.msg.html

(cl:defclass <GetJointPosition-response> (roslisp-msg-protocol:ros-message)
  ((joint_position
    :reader joint_position
    :initarg :joint_position
    :type open_manipulator_msgs-msg:JointPosition
    :initform (cl:make-instance 'open_manipulator_msgs-msg:JointPosition)))
)

(cl:defclass GetJointPosition-response (<GetJointPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetJointPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetJointPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name open_manipulator_msgs-srv:<GetJointPosition-response> is deprecated: use open_manipulator_msgs-srv:GetJointPosition-response instead.")))

(cl:ensure-generic-function 'joint_position-val :lambda-list '(m))
(cl:defmethod joint_position-val ((m <GetJointPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_manipulator_msgs-srv:joint_position-val is deprecated.  Use open_manipulator_msgs-srv:joint_position instead.")
  (joint_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetJointPosition-response>) ostream)
  "Serializes a message object of type '<GetJointPosition-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetJointPosition-response>) istream)
  "Deserializes a message object of type '<GetJointPosition-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetJointPosition-response>)))
  "Returns string type for a service object of type '<GetJointPosition-response>"
  "open_manipulator_msgs/GetJointPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointPosition-response)))
  "Returns string type for a service object of type 'GetJointPosition-response"
  "open_manipulator_msgs/GetJointPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetJointPosition-response>)))
  "Returns md5sum for a message object of type '<GetJointPosition-response>"
  "e1f1ee99b5e77308297dc4eeedd305d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetJointPosition-response)))
  "Returns md5sum for a message object of type 'GetJointPosition-response"
  "e1f1ee99b5e77308297dc4eeedd305d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetJointPosition-response>)))
  "Returns full string definition for message of type '<GetJointPosition-response>"
  (cl:format cl:nil "JointPosition joint_position~%~%~%~%================================================================================~%MSG: open_manipulator_msgs/JointPosition~%string[]   joint_name~%float64[]  position~%float64    max_accelerations_scaling_factor~%float64    max_velocity_scaling_factor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetJointPosition-response)))
  "Returns full string definition for message of type 'GetJointPosition-response"
  (cl:format cl:nil "JointPosition joint_position~%~%~%~%================================================================================~%MSG: open_manipulator_msgs/JointPosition~%string[]   joint_name~%float64[]  position~%float64    max_accelerations_scaling_factor~%float64    max_velocity_scaling_factor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetJointPosition-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetJointPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetJointPosition-response
    (cl:cons ':joint_position (joint_position msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetJointPosition)))
  'GetJointPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetJointPosition)))
  'GetJointPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointPosition)))
  "Returns string type for a service object of type '<GetJointPosition>"
  "open_manipulator_msgs/GetJointPosition")