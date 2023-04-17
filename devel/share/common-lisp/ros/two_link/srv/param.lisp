; Auto-generated. Do not edit!


(cl:in-package two_link-srv)


;//! \htmlinclude param-request.msg.html

(cl:defclass <param-request> (roslisp-msg-protocol:ros-message)
  ((amplitude
    :reader amplitude
    :initarg :amplitude
    :type cl:float
    :initform 0.0)
   (period
    :reader period
    :initarg :period
    :type cl:float
    :initform 0.0))
)

(cl:defclass param-request (<param-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <param-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'param-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name two_link-srv:<param-request> is deprecated: use two_link-srv:param-request instead.")))

(cl:ensure-generic-function 'amplitude-val :lambda-list '(m))
(cl:defmethod amplitude-val ((m <param-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader two_link-srv:amplitude-val is deprecated.  Use two_link-srv:amplitude instead.")
  (amplitude m))

(cl:ensure-generic-function 'period-val :lambda-list '(m))
(cl:defmethod period-val ((m <param-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader two_link-srv:period-val is deprecated.  Use two_link-srv:period instead.")
  (period m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <param-request>) ostream)
  "Serializes a message object of type '<param-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'amplitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'period))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <param-request>) istream)
  "Deserializes a message object of type '<param-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'amplitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'period) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<param-request>)))
  "Returns string type for a service object of type '<param-request>"
  "two_link/paramRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'param-request)))
  "Returns string type for a service object of type 'param-request"
  "two_link/paramRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<param-request>)))
  "Returns md5sum for a message object of type '<param-request>"
  "da3e0ad9469889454218ee3b1f92c73a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'param-request)))
  "Returns md5sum for a message object of type 'param-request"
  "da3e0ad9469889454218ee3b1f92c73a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<param-request>)))
  "Returns full string definition for message of type '<param-request>"
  (cl:format cl:nil "float64 amplitude~%float64 period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'param-request)))
  "Returns full string definition for message of type 'param-request"
  (cl:format cl:nil "float64 amplitude~%float64 period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <param-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <param-request>))
  "Converts a ROS message object to a list"
  (cl:list 'param-request
    (cl:cons ':amplitude (amplitude msg))
    (cl:cons ':period (period msg))
))
;//! \htmlinclude param-response.msg.html

(cl:defclass <param-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass param-response (<param-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <param-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'param-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name two_link-srv:<param-response> is deprecated: use two_link-srv:param-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <param-response>) ostream)
  "Serializes a message object of type '<param-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <param-response>) istream)
  "Deserializes a message object of type '<param-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<param-response>)))
  "Returns string type for a service object of type '<param-response>"
  "two_link/paramResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'param-response)))
  "Returns string type for a service object of type 'param-response"
  "two_link/paramResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<param-response>)))
  "Returns md5sum for a message object of type '<param-response>"
  "da3e0ad9469889454218ee3b1f92c73a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'param-response)))
  "Returns md5sum for a message object of type 'param-response"
  "da3e0ad9469889454218ee3b1f92c73a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<param-response>)))
  "Returns full string definition for message of type '<param-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'param-response)))
  "Returns full string definition for message of type 'param-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <param-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <param-response>))
  "Converts a ROS message object to a list"
  (cl:list 'param-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'param)))
  'param-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'param)))
  'param-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'param)))
  "Returns string type for a service object of type '<param>"
  "two_link/param")