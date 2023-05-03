
(cl:in-package :asdf)

(defsystem "dynamixel_workbench_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :dynamixel_workbench_msgs-msg
)
  :components ((:file "_package")
    (:file "DynamixelCommand" :depends-on ("_package_DynamixelCommand"))
    (:file "_package_DynamixelCommand" :depends-on ("_package"))
    (:file "EECommand" :depends-on ("_package_EECommand"))
    (:file "_package_EECommand" :depends-on ("_package"))
    (:file "GetDynamixelInfo" :depends-on ("_package_GetDynamixelInfo"))
    (:file "_package_GetDynamixelInfo" :depends-on ("_package"))
    (:file "JointCommand" :depends-on ("_package_JointCommand"))
    (:file "_package_JointCommand" :depends-on ("_package"))
    (:file "WheelCommand" :depends-on ("_package_WheelCommand"))
    (:file "_package_WheelCommand" :depends-on ("_package"))
    (:file "test" :depends-on ("_package_test"))
    (:file "_package_test" :depends-on ("_package"))
  ))