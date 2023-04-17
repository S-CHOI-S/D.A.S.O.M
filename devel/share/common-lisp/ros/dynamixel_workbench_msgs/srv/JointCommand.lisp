; Auto-generated. Do not edit!


(cl:in-package dynamixel_workbench_msgs-srv)


;//! \htmlinclude JointCommand-request.msg.html

(cl:defclass <JointCommand-request> (roslisp-msg-protocol:ros-message)
  ((unit
    :reader unit
    :initarg :unit
    :type cl:string
    :initform "")
   (id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (goal_position
    :reader goal_position
    :initarg :goal_position
    :type cl:float
    :initform 0.0)
   (goal_position_2
    :reader goal_position_2
    :initarg :goal_position_2
    :type cl:float
    :initform 0.0))
)

(cl:defclass JointCommand-request (<JointCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_workbench_msgs-srv:<JointCommand-request> is deprecated: use dynamixel_workbench_msgs-srv:JointCommand-request instead.")))

(cl:ensure-generic-function 'unit-val :lambda-list '(m))
(cl:defmethod unit-val ((m <JointCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_workbench_msgs-srv:unit-val is deprecated.  Use dynamixel_workbench_msgs-srv:unit instead.")
  (unit m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <JointCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_workbench_msgs-srv:id-val is deprecated.  Use dynamixel_workbench_msgs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'goal_position-val :lambda-list '(m))
(cl:defmethod goal_position-val ((m <JointCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_workbench_msgs-srv:goal_position-val is deprecated.  Use dynamixel_workbench_msgs-srv:goal_position instead.")
  (goal_position m))

(cl:ensure-generic-function 'goal_position_2-val :lambda-list '(m))
(cl:defmethod goal_position_2-val ((m <JointCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_workbench_msgs-srv:goal_position_2-val is deprecated.  Use dynamixel_workbench_msgs-srv:goal_position_2 instead.")
  (goal_position_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointCommand-request>) ostream)
  "Serializes a message object of type '<JointCommand-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'unit))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'unit))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'goal_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'goal_position_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointCommand-request>) istream)
  "Deserializes a message object of type '<JointCommand-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'unit) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'unit) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal_position) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal_position_2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointCommand-request>)))
  "Returns string type for a service object of type '<JointCommand-request>"
  "dynamixel_workbench_msgs/JointCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointCommand-request)))
  "Returns string type for a service object of type 'JointCommand-request"
  "dynamixel_workbench_msgs/JointCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointCommand-request>)))
  "Returns md5sum for a message object of type '<JointCommand-request>"
  "0f59c59a18a44b029e233012bc29d67d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointCommand-request)))
  "Returns md5sum for a message object of type 'JointCommand-request"
  "0f59c59a18a44b029e233012bc29d67d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointCommand-request>)))
  "Returns full string definition for message of type '<JointCommand-request>"
  (cl:format cl:nil "# This message is used to send position command to dynamixel~%~%string unit~%uint8 id~%float32 goal_position~%float32 goal_position_2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointCommand-request)))
  "Returns full string definition for message of type 'JointCommand-request"
  (cl:format cl:nil "# This message is used to send position command to dynamixel~%~%string unit~%uint8 id~%float32 goal_position~%float32 goal_position_2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointCommand-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'unit))
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'JointCommand-request
    (cl:cons ':unit (unit msg))
    (cl:cons ':id (id msg))
    (cl:cons ':goal_position (goal_position msg))
    (cl:cons ':goal_position_2 (goal_position_2 msg))
))
;//! \htmlinclude JointCommand-response.msg.html

(cl:defclass <JointCommand-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass JointCommand-response (<JointCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_workbench_msgs-srv:<JointCommand-response> is deprecated: use dynamixel_workbench_msgs-srv:JointCommand-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <JointCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_workbench_msgs-srv:result-val is deprecated.  Use dynamixel_workbench_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointCommand-response>) ostream)
  "Serializes a message object of type '<JointCommand-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointCommand-response>) istream)
  "Deserializes a message object of type '<JointCommand-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointCommand-response>)))
  "Returns string type for a service object of type '<JointCommand-response>"
  "dynamixel_workbench_msgs/JointCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointCommand-response)))
  "Returns string type for a service object of type 'JointCommand-response"
  "dynamixel_workbench_msgs/JointCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointCommand-response>)))
  "Returns md5sum for a message object of type '<JointCommand-response>"
  "0f59c59a18a44b029e233012bc29d67d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointCommand-response)))
  "Returns md5sum for a message object of type 'JointCommand-response"
  "0f59c59a18a44b029e233012bc29d67d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointCommand-response>)))
  "Returns full string definition for message of type '<JointCommand-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointCommand-response)))
  "Returns full string definition for message of type 'JointCommand-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointCommand-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'JointCommand-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'JointCommand)))
  'JointCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'JointCommand)))
  'JointCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointCommand)))
  "Returns string type for a service object of type '<JointCommand>"
  "dynamixel_workbench_msgs/JointCommand")