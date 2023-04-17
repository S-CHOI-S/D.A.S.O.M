
(cl:in-package :asdf)

(defsystem "two_link-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DasomDynamixel" :depends-on ("_package_DasomDynamixel"))
    (:file "_package_DasomDynamixel" :depends-on ("_package"))
  ))