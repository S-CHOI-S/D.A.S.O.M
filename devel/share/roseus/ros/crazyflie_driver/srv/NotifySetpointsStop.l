;; Auto-generated. Do not edit!


(when (boundp 'crazyflie_driver::NotifySetpointsStop)
  (if (not (find-package "CRAZYFLIE_DRIVER"))
    (make-package "CRAZYFLIE_DRIVER"))
  (shadow 'NotifySetpointsStop (find-package "CRAZYFLIE_DRIVER")))
(unless (find-package "CRAZYFLIE_DRIVER::NOTIFYSETPOINTSSTOP")
  (make-package "CRAZYFLIE_DRIVER::NOTIFYSETPOINTSSTOP"))
(unless (find-package "CRAZYFLIE_DRIVER::NOTIFYSETPOINTSSTOPREQUEST")
  (make-package "CRAZYFLIE_DRIVER::NOTIFYSETPOINTSSTOPREQUEST"))
(unless (find-package "CRAZYFLIE_DRIVER::NOTIFYSETPOINTSSTOPRESPONSE")
  (make-package "CRAZYFLIE_DRIVER::NOTIFYSETPOINTSSTOPRESPONSE"))

(in-package "ROS")





(defclass crazyflie_driver::NotifySetpointsStopRequest
  :super ros::object
  :slots (_groupMask _remainValidMillisecs ))

(defmethod crazyflie_driver::NotifySetpointsStopRequest
  (:init
   (&key
    ((:groupMask __groupMask) 0)
    ((:remainValidMillisecs __remainValidMillisecs) 0)
    )
   (send-super :init)
   (setq _groupMask (round __groupMask))
   (setq _remainValidMillisecs (round __remainValidMillisecs))
   self)
  (:groupMask
   (&optional __groupMask)
   (if __groupMask (setq _groupMask __groupMask)) _groupMask)
  (:remainValidMillisecs
   (&optional __remainValidMillisecs)
   (if __remainValidMillisecs (setq _remainValidMillisecs __remainValidMillisecs)) _remainValidMillisecs)
  (:serialization-length
   ()
   (+
    ;; uint8 _groupMask
    1
    ;; uint32 _remainValidMillisecs
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _groupMask
       (write-byte _groupMask s)
     ;; uint32 _remainValidMillisecs
       (write-long _remainValidMillisecs s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _groupMask
     (setq _groupMask (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _remainValidMillisecs
     (setq _remainValidMillisecs (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass crazyflie_driver::NotifySetpointsStopResponse
  :super ros::object
  :slots ())

(defmethod crazyflie_driver::NotifySetpointsStopResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass crazyflie_driver::NotifySetpointsStop
  :super ros::object
  :slots ())

(setf (get crazyflie_driver::NotifySetpointsStop :md5sum-) "5e1e40fcf516d3883bc228627d3e1e43")
(setf (get crazyflie_driver::NotifySetpointsStop :datatype-) "crazyflie_driver/NotifySetpointsStop")
(setf (get crazyflie_driver::NotifySetpointsStop :request) crazyflie_driver::NotifySetpointsStopRequest)
(setf (get crazyflie_driver::NotifySetpointsStop :response) crazyflie_driver::NotifySetpointsStopResponse)

(defmethod crazyflie_driver::NotifySetpointsStopRequest
  (:response () (instance crazyflie_driver::NotifySetpointsStopResponse :init)))

(setf (get crazyflie_driver::NotifySetpointsStopRequest :md5sum-) "5e1e40fcf516d3883bc228627d3e1e43")
(setf (get crazyflie_driver::NotifySetpointsStopRequest :datatype-) "crazyflie_driver/NotifySetpointsStopRequest")
(setf (get crazyflie_driver::NotifySetpointsStopRequest :definition-)
      "uint8 groupMask
uint32 remainValidMillisecs
---

")

(setf (get crazyflie_driver::NotifySetpointsStopResponse :md5sum-) "5e1e40fcf516d3883bc228627d3e1e43")
(setf (get crazyflie_driver::NotifySetpointsStopResponse :datatype-) "crazyflie_driver/NotifySetpointsStopResponse")
(setf (get crazyflie_driver::NotifySetpointsStopResponse :definition-)
      "uint8 groupMask
uint32 remainValidMillisecs
---

")



(provide :crazyflie_driver/NotifySetpointsStop "5e1e40fcf516d3883bc228627d3e1e43")


