;; Auto-generated. Do not edit!


(when (boundp 'two_link::param)
  (if (not (find-package "TWO_LINK"))
    (make-package "TWO_LINK"))
  (shadow 'param (find-package "TWO_LINK")))
(unless (find-package "TWO_LINK::PARAM")
  (make-package "TWO_LINK::PARAM"))
(unless (find-package "TWO_LINK::PARAMREQUEST")
  (make-package "TWO_LINK::PARAMREQUEST"))
(unless (find-package "TWO_LINK::PARAMRESPONSE")
  (make-package "TWO_LINK::PARAMRESPONSE"))

(in-package "ROS")





(defclass two_link::paramRequest
  :super ros::object
  :slots (_amplitude _period ))

(defmethod two_link::paramRequest
  (:init
   (&key
    ((:amplitude __amplitude) 0.0)
    ((:period __period) 0.0)
    )
   (send-super :init)
   (setq _amplitude (float __amplitude))
   (setq _period (float __period))
   self)
  (:amplitude
   (&optional __amplitude)
   (if __amplitude (setq _amplitude __amplitude)) _amplitude)
  (:period
   (&optional __period)
   (if __period (setq _period __period)) _period)
  (:serialization-length
   ()
   (+
    ;; float64 _amplitude
    8
    ;; float64 _period
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _amplitude
       (sys::poke _amplitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _period
       (sys::poke _period (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _amplitude
     (setq _amplitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _period
     (setq _period (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass two_link::paramResponse
  :super ros::object
  :slots ())

(defmethod two_link::paramResponse
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

(defclass two_link::param
  :super ros::object
  :slots ())

(setf (get two_link::param :md5sum-) "da3e0ad9469889454218ee3b1f92c73a")
(setf (get two_link::param :datatype-) "two_link/param")
(setf (get two_link::param :request) two_link::paramRequest)
(setf (get two_link::param :response) two_link::paramResponse)

(defmethod two_link::paramRequest
  (:response () (instance two_link::paramResponse :init)))

(setf (get two_link::paramRequest :md5sum-) "da3e0ad9469889454218ee3b1f92c73a")
(setf (get two_link::paramRequest :datatype-) "two_link/paramRequest")
(setf (get two_link::paramRequest :definition-)
      "float64 amplitude
float64 period
---

")

(setf (get two_link::paramResponse :md5sum-) "da3e0ad9469889454218ee3b1f92c73a")
(setf (get two_link::paramResponse :datatype-) "two_link/paramResponse")
(setf (get two_link::paramResponse :definition-)
      "float64 amplitude
float64 period
---

")



(provide :two_link/param "da3e0ad9469889454218ee3b1f92c73a")


