
(cl:in-package :asdf)

(defsystem "test-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Torqbian" :depends-on ("_package_Torqbian"))
    (:file "_package_Torqbian" :depends-on ("_package"))
  ))