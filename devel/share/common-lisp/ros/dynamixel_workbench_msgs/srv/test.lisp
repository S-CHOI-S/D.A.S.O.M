; Auto-generated. Do not edit!


(cl:in-package dynamixel_workbench_msgs-srv)


;//! \htmlinclude test-request.msg.html

(cl:defclass <test-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass test-request (<test-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <test-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'test-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_workbench_msgs-srv:<test-request> is deprecated: use dynamixel_workbench_msgs-srv:test-request instead.")))

(cl:ensure-generic-function 'right_vel-val :lambda-list '(m))
(cl:defmethod right_vel-val ((m <test-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_workbench_msgs-srv:right_vel-val is deprecated.  Use dynamixel_workbench_msgs-srv:right_vel instead.")
  (right_vel m))

(cl:ensure-generic-function 'left_vel-val :lambda-list '(m))
(cl:defmethod left_vel-val ((m <test-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_workbench_msgs-srv:left_vel-val is deprecated.  Use dynamixel_workbench_msgs-srv:left_vel instead.")
  (left_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <test-request>) ostream)
  "Serializes a message object of type '<test-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <test-request>) istream)
  "Deserializes a message object of type '<test-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<test-request>)))
  "Returns string type for a service object of type '<test-request>"
  "dynamixel_workbench_msgs/testRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'test-request)))
  "Returns string type for a service object of type 'test-request"
  "dynamixel_workbench_msgs/testRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<test-request>)))
  "Returns md5sum for a message object of type '<test-request>"
  "a7851f940124222501142592f81f3c11")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'test-request)))
  "Returns md5sum for a message object of type 'test-request"
  "a7851f940124222501142592f81f3c11")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<test-request>)))
  "Returns full string definition for message of type '<test-request>"
  (cl:format cl:nil "# This message is used to send velocity command to dynamixel~%~%float32 right_vel~%float32 left_vel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'test-request)))
  "Returns full string definition for message of type 'test-request"
  (cl:format cl:nil "# This message is used to send velocity command to dynamixel~%~%float32 right_vel~%float32 left_vel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <test-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <test-request>))
  "Converts a ROS message object to a list"
  (cl:list 'test-request
    (cl:cons ':right_vel (right_vel msg))
    (cl:cons ':left_vel (left_vel msg))
))
;//! \htmlinclude test-response.msg.html

(cl:defclass <test-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass test-response (<test-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <test-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'test-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_workbench_msgs-srv:<test-response> is deprecated: use dynamixel_workbench_msgs-srv:test-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <test-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_workbench_msgs-srv:result-val is deprecated.  Use dynamixel_workbench_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <test-response>) ostream)
  "Serializes a message object of type '<test-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <test-response>) istream)
  "Deserializes a message object of type '<test-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<test-response>)))
  "Returns string type for a service object of type '<test-response>"
  "dynamixel_workbench_msgs/testResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'test-response)))
  "Returns string type for a service object of type 'test-response"
  "dynamixel_workbench_msgs/testResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<test-response>)))
  "Returns md5sum for a message object of type '<test-response>"
  "a7851f940124222501142592f81f3c11")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'test-response)))
  "Returns md5sum for a message object of type 'test-response"
  "a7851f940124222501142592f81f3c11")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<test-response>)))
  "Returns full string definition for message of type '<test-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'test-response)))
  "Returns full string definition for message of type 'test-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <test-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <test-response>))
  "Converts a ROS message object to a list"
  (cl:list 'test-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'test)))
  'test-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'test)))
  'test-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'test)))
  "Returns string type for a service object of type '<test>"
  "dynamixel_workbench_msgs/test")