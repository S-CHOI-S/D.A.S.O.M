;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_workbench_msgs::DynamixelCommand)
  (if (not (find-package "DYNAMIXEL_WORKBENCH_MSGS"))
    (make-package "DYNAMIXEL_WORKBENCH_MSGS"))
  (shadow 'DynamixelCommand (find-package "DYNAMIXEL_WORKBENCH_MSGS")))
(unless (find-package "DYNAMIXEL_WORKBENCH_MSGS::DYNAMIXELCOMMAND")
  (make-package "DYNAMIXEL_WORKBENCH_MSGS::DYNAMIXELCOMMAND"))
(unless (find-package "DYNAMIXEL_WORKBENCH_MSGS::DYNAMIXELCOMMANDREQUEST")
  (make-package "DYNAMIXEL_WORKBENCH_MSGS::DYNAMIXELCOMMANDREQUEST"))
(unless (find-package "DYNAMIXEL_WORKBENCH_MSGS::DYNAMIXELCOMMANDRESPONSE")
  (make-package "DYNAMIXEL_WORKBENCH_MSGS::DYNAMIXELCOMMANDRESPONSE"))

(in-package "ROS")





(defclass dynamixel_workbench_msgs::DynamixelCommandRequest
  :super ros::object
  :slots (_command _addr_name _value ))

(defmethod dynamixel_workbench_msgs::DynamixelCommandRequest
  (:init
   (&key
    ((:command __command) "")
    ((:addr_name __addr_name) "")
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _command (string __command))
   (setq _addr_name (string __addr_name))
   (setq _value (round __value))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:addr_name
   (&optional __addr_name)
   (if __addr_name (setq _addr_name __addr_name)) _addr_name)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ;; string _addr_name
    4 (length _addr_name)
    ;; int64 _value
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;; string _addr_name
       (write-long (length _addr_name) s) (princ _addr_name s)
     ;; int64 _value
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _value (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _value) (= (length (_value . bv)) 2)) ;; bignum
              (write-long (ash (elt (_value . bv) 0) 0) s)
              (write-long (ash (elt (_value . bv) 1) -1) s))
             ((and (class _value) (= (length (_value . bv)) 1)) ;; big1
              (write-long (elt (_value . bv) 0) s)
              (write-long (if (>= _value 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _value s)(write-long (if (>= _value 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _addr_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _addr_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _value
#+(or :alpha :irix6 :x86_64)
      (setf _value (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _value (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass dynamixel_workbench_msgs::DynamixelCommandResponse
  :super ros::object
  :slots (_comm_result ))

(defmethod dynamixel_workbench_msgs::DynamixelCommandResponse
  (:init
   (&key
    ((:comm_result __comm_result) nil)
    )
   (send-super :init)
   (setq _comm_result __comm_result)
   self)
  (:comm_result
   (&optional __comm_result)
   (if __comm_result (setq _comm_result __comm_result)) _comm_result)
  (:serialization-length
   ()
   (+
    ;; bool _comm_result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _comm_result
       (if _comm_result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _comm_result
     (setq _comm_result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dynamixel_workbench_msgs::DynamixelCommand
  :super ros::object
  :slots ())

(setf (get dynamixel_workbench_msgs::DynamixelCommand :md5sum-) "cf973b38a14a7c815992bad0743e3f5b")
(setf (get dynamixel_workbench_msgs::DynamixelCommand :datatype-) "dynamixel_workbench_msgs/DynamixelCommand")
(setf (get dynamixel_workbench_msgs::DynamixelCommand :request) dynamixel_workbench_msgs::DynamixelCommandRequest)
(setf (get dynamixel_workbench_msgs::DynamixelCommand :response) dynamixel_workbench_msgs::DynamixelCommandResponse)

(defmethod dynamixel_workbench_msgs::DynamixelCommandRequest
  (:response () (instance dynamixel_workbench_msgs::DynamixelCommandResponse :init)))

(setf (get dynamixel_workbench_msgs::DynamixelCommandRequest :md5sum-) "cf973b38a14a7c815992bad0743e3f5b")
(setf (get dynamixel_workbench_msgs::DynamixelCommandRequest :datatype-) "dynamixel_workbench_msgs/DynamixelCommandRequest")
(setf (get dynamixel_workbench_msgs::DynamixelCommandRequest :definition-)
      "# It is Served by 'single_dynamixel_monitor' node
# This message is used to send command to dynamixel

string command

string addr_name
int64 value

---

bool comm_result

")

(setf (get dynamixel_workbench_msgs::DynamixelCommandResponse :md5sum-) "cf973b38a14a7c815992bad0743e3f5b")
(setf (get dynamixel_workbench_msgs::DynamixelCommandResponse :datatype-) "dynamixel_workbench_msgs/DynamixelCommandResponse")
(setf (get dynamixel_workbench_msgs::DynamixelCommandResponse :definition-)
      "# It is Served by 'single_dynamixel_monitor' node
# This message is used to send command to dynamixel

string command

string addr_name
int64 value

---

bool comm_result

")



(provide :dynamixel_workbench_msgs/DynamixelCommand "cf973b38a14a7c815992bad0743e3f5b")


