; Auto-generated. Do not edit!


(cl:in-package dynamixel_workbench_msgs-srv)


;//! \htmlinclude EECommand-request.msg.html

(cl:defclass <EECommand-request> (roslisp-msg-protocol:ros-message)
  ((X
    :reader X
    :initarg :X
    :type cl:float
    :initform 0.0)
   (Y
    :reader Y
    :initarg :Y
    :type cl:float
    :initform 0.0))
)

(cl:defclass EECommand-request (<EECommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EECommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EECommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_workbench_msgs-srv:<EECommand-request> is deprecated: use dynamixel_workbench_msgs-srv:EECommand-request instead.")))

(cl:ensure-generic-function 'X-val :lambda-list '(m))
(cl:defmethod X-val ((m <EECommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_workbench_msgs-srv:X-val is deprecated.  Use dynamixel_workbench_msgs-srv:X instead.")
  (X m))

(cl:ensure-generic-function 'Y-val :lambda-list '(m))
(cl:defmethod Y-val ((m <EECommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_workbench_msgs-srv:Y-val is deprecated.  Use dynamixel_workbench_msgs-srv:Y instead.")
  (Y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EECommand-request>) ostream)
  "Serializes a message object of type '<EECommand-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'X))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EECommand-request>) istream)
  "Deserializes a message object of type '<EECommand-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'X) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Y) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EECommand-request>)))
  "Returns string type for a service object of type '<EECommand-request>"
  "dynamixel_workbench_msgs/EECommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EECommand-request)))
  "Returns string type for a service object of type 'EECommand-request"
  "dynamixel_workbench_msgs/EECommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EECommand-request>)))
  "Returns md5sum for a message object of type '<EECommand-request>"
  "e7f17a0f3bbae3e0b8d5b7c65426d275")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EECommand-request)))
  "Returns md5sum for a message object of type 'EECommand-request"
  "e7f17a0f3bbae3e0b8d5b7c65426d275")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EECommand-request>)))
  "Returns full string definition for message of type '<EECommand-request>"
  (cl:format cl:nil "# This message is used to send End-Effector command to DASOM~%~%float64 X~%float64 Y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EECommand-request)))
  "Returns full string definition for message of type 'EECommand-request"
  (cl:format cl:nil "# This message is used to send End-Effector command to DASOM~%~%float64 X~%float64 Y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EECommand-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EECommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EECommand-request
    (cl:cons ':X (X msg))
    (cl:cons ':Y (Y msg))
))
;//! \htmlinclude EECommand-response.msg.html

(cl:defclass <EECommand-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass EECommand-response (<EECommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EECommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EECommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_workbench_msgs-srv:<EECommand-response> is deprecated: use dynamixel_workbench_msgs-srv:EECommand-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EECommand-response>) ostream)
  "Serializes a message object of type '<EECommand-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EECommand-response>) istream)
  "Deserializes a message object of type '<EECommand-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EECommand-response>)))
  "Returns string type for a service object of type '<EECommand-response>"
  "dynamixel_workbench_msgs/EECommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EECommand-response)))
  "Returns string type for a service object of type 'EECommand-response"
  "dynamixel_workbench_msgs/EECommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EECommand-response>)))
  "Returns md5sum for a message object of type '<EECommand-response>"
  "e7f17a0f3bbae3e0b8d5b7c65426d275")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EECommand-response)))
  "Returns md5sum for a message object of type 'EECommand-response"
  "e7f17a0f3bbae3e0b8d5b7c65426d275")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EECommand-response>)))
  "Returns full string definition for message of type '<EECommand-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EECommand-response)))
  "Returns full string definition for message of type 'EECommand-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EECommand-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EECommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EECommand-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EECommand)))
  'EECommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EECommand)))
  'EECommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EECommand)))
  "Returns string type for a service object of type '<EECommand>"
  "dynamixel_workbench_msgs/EECommand")