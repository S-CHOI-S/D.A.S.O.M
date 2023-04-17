; Auto-generated. Do not edit!


(cl:in-package dynamixel_workbench_msgs-srv)


;//! \htmlinclude WheelCommand-request.msg.html

(cl:defclass <WheelCommand-request> (roslisp-msg-protocol:ros-message)
  ((right_vel
    :reader right_vel
    :initarg :right_vel
    :type cl:float
    :initform 0.0)
   (left_vel
    :reader left_vel
    :initarg :left_vel
    :type cl:float
    :initform 0.0))
)

(cl:defclass WheelCommand-request (<WheelCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_workbench_msgs-srv:<WheelCommand-request> is deprecated: use dynamixel_workbench_msgs-srv:WheelCommand-request instead.")))

(cl:ensure-generic-function 'right_vel-val :lambda-list '(m))
(cl:defmethod right_vel-val ((m <WheelCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_workbench_msgs-srv:right_vel-val is deprecated.  Use dynamixel_workbench_msgs-srv:right_vel instead.")
  (right_vel m))

(cl:ensure-generic-function 'left_vel-val :lambda-list '(m))
(cl:defmethod left_vel-val ((m <WheelCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_workbench_msgs-srv:left_vel-val is deprecated.  Use dynamixel_workbench_msgs-srv:left_vel instead.")
  (left_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelCommand-request>) ostream)
  "Serializes a message object of type '<WheelCommand-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelCommand-request>) istream)
  "Deserializes a message object of type '<WheelCommand-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_vel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_vel) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelCommand-request>)))
  "Returns string type for a service object of type '<WheelCommand-request>"
  "dynamixel_workbench_msgs/WheelCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelCommand-request)))
  "Returns string type for a service object of type 'WheelCommand-request"
  "dynamixel_workbench_msgs/WheelCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelCommand-request>)))
  "Returns md5sum for a message object of type '<WheelCommand-request>"
  "a7851f940124222501142592f81f3c11")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelCommand-request)))
  "Returns md5sum for a message object of type 'WheelCommand-request"
  "a7851f940124222501142592f81f3c11")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelCommand-request>)))
  "Returns full string definition for message of type '<WheelCommand-request>"
  (cl:format cl:nil "# This message is used to send velocity command to dynamixel~%~%float32 right_vel~%float32 left_vel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelCommand-request)))
  "Returns full string definition for message of type 'WheelCommand-request"
  (cl:format cl:nil "# This message is used to send velocity command to dynamixel~%~%float32 right_vel~%float32 left_vel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelCommand-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelCommand-request
    (cl:cons ':right_vel (right_vel msg))
    (cl:cons ':left_vel (left_vel msg))
))
;//! \htmlinclude WheelCommand-response.msg.html

(cl:defclass <WheelCommand-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass WheelCommand-response (<WheelCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_workbench_msgs-srv:<WheelCommand-response> is deprecated: use dynamixel_workbench_msgs-srv:WheelCommand-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <WheelCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_workbench_msgs-srv:result-val is deprecated.  Use dynamixel_workbench_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelCommand-response>) ostream)
  "Serializes a message object of type '<WheelCommand-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelCommand-response>) istream)
  "Deserializes a message object of type '<WheelCommand-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelCommand-response>)))
  "Returns string type for a service object of type '<WheelCommand-response>"
  "dynamixel_workbench_msgs/WheelCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelCommand-response)))
  "Returns string type for a service object of type 'WheelCommand-response"
  "dynamixel_workbench_msgs/WheelCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelCommand-response>)))
  "Returns md5sum for a message object of type '<WheelCommand-response>"
  "a7851f940124222501142592f81f3c11")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelCommand-response)))
  "Returns md5sum for a message object of type 'WheelCommand-response"
  "a7851f940124222501142592f81f3c11")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelCommand-response>)))
  "Returns full string definition for message of type '<WheelCommand-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelCommand-response)))
  "Returns full string definition for message of type 'WheelCommand-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelCommand-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelCommand-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WheelCommand)))
  'WheelCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WheelCommand)))
  'WheelCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelCommand)))
  "Returns string type for a service object of type '<WheelCommand>"
  "dynamixel_workbench_msgs/WheelCommand")