
(cl:in-package :asdf)

(defsystem "open_manipulator_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :open_manipulator_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GetJointPosition" :depends-on ("_package_GetJointPosition"))
    (:file "_package_GetJointPosition" :depends-on ("_package"))
    (:file "GetKinematicsPose" :depends-on ("_package_GetKinematicsPose"))
    (:file "_package_GetKinematicsPose" :depends-on ("_package"))
    (:file "SetJointPosition" :depends-on ("_package_SetJointPosition"))
    (:file "_package_SetJointPosition" :depends-on ("_package"))
    (:file "SetKinematicsPose" :depends-on ("_package_SetKinematicsPose"))
    (:file "_package_SetKinematicsPose" :depends-on ("_package"))
  ))