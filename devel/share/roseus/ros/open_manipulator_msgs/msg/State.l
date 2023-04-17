;; Auto-generated. Do not edit!


(when (boundp 'open_manipulator_msgs::State)
  (if (not (find-package "OPEN_MANIPULATOR_MSGS"))
    (make-package "OPEN_MANIPULATOR_MSGS"))
  (shadow 'State (find-package "OPEN_MANIPULATOR_MSGS")))
(unless (find-package "OPEN_MANIPULATOR_MSGS::STATE")
  (make-package "OPEN_MANIPULATOR_MSGS::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html


(intern "*IS_MOVING*" (find-package "OPEN_MANIPULATOR_MSGS::STATE"))
(shadow '*IS_MOVING* (find-package "OPEN_MANIPULATOR_MSGS::STATE"))
(defconstant open_manipulator_msgs::State::*IS_MOVING* "\"IS_MOVING\"")
(intern "*STOPPED*" (find-package "OPEN_MANIPULATOR_MSGS::STATE"))
(shadow '*STOPPED* (find-package "OPEN_MANIPULATOR_MSGS::STATE"))
(defconstant open_manipulator_msgs::State::*STOPPED* "\"STOPPED\"")
(defclass open_manipulator_msgs::State
  :super ros::object
  :slots (_robot ))

(defmethod open_manipulator_msgs::State
  (:init
   (&key
    ((:robot __robot) "")
    )
   (send-super :init)
   (setq _robot (string __robot))
   self)
  (:robot
   (&optional __robot)
   (if __robot (setq _robot __robot)) _robot)
  (:serialization-length
   ()
   (+
    ;; string _robot
    4 (length _robot)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _robot
       (write-long (length _robot) s) (princ _robot s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _robot
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get open_manipulator_msgs::State :md5sum-) "94370aa4b03eef51ce4d539f789d976f")
(setf (get open_manipulator_msgs::State :datatype-) "open_manipulator_msgs/State")
(setf (get open_manipulator_msgs::State :definition-)
      "########################################
# CONSTANTS
########################################
string IS_MOVING = \"IS_MOVING\"
string STOPPED = \"STOPPED\"

########################################
# Messages
########################################
string robot

")



(provide :open_manipulator_msgs/State "94370aa4b03eef51ce4d539f789d976f")


