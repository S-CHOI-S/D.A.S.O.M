
(cl:in-package :asdf)

(defsystem "dynamixel_workbench_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AX" :depends-on ("_package_AX"))
    (:file "_package_AX" :depends-on ("_package"))
    (:file "DasomDynamixel" :depends-on ("_package_DasomDynamixel"))
    (:file "_package_DasomDynamixel" :depends-on ("_package"))
    (:file "DynamixelInfo" :depends-on ("_package_DynamixelInfo"))
    (:file "_package_DynamixelInfo" :depends-on ("_package"))
    (:file "DynamixelLoadInfo" :depends-on ("_package_DynamixelLoadInfo"))
    (:file "_package_DynamixelLoadInfo" :depends-on ("_package"))
    (:file "DynamixelState" :depends-on ("_package_DynamixelState"))
    (:file "_package_DynamixelState" :depends-on ("_package"))
    (:file "DynamixelStateList" :depends-on ("_package_DynamixelStateList"))
    (:file "_package_DynamixelStateList" :depends-on ("_package"))
    (:file "EX" :depends-on ("_package_EX"))
    (:file "_package_EX" :depends-on ("_package"))
    (:file "MX" :depends-on ("_package_MX"))
    (:file "_package_MX" :depends-on ("_package"))
    (:file "MX2" :depends-on ("_package_MX2"))
    (:file "_package_MX2" :depends-on ("_package"))
    (:file "MX2Ext" :depends-on ("_package_MX2Ext"))
    (:file "_package_MX2Ext" :depends-on ("_package"))
    (:file "MXExt" :depends-on ("_package_MXExt"))
    (:file "_package_MXExt" :depends-on ("_package"))
    (:file "PRO" :depends-on ("_package_PRO"))
    (:file "_package_PRO" :depends-on ("_package"))
    (:file "PROExt" :depends-on ("_package_PROExt"))
    (:file "_package_PROExt" :depends-on ("_package"))
    (:file "RX" :depends-on ("_package_RX"))
    (:file "_package_RX" :depends-on ("_package"))
    (:file "XH" :depends-on ("_package_XH"))
    (:file "_package_XH" :depends-on ("_package"))
    (:file "XL" :depends-on ("_package_XL"))
    (:file "_package_XL" :depends-on ("_package"))
    (:file "XL320" :depends-on ("_package_XL320"))
    (:file "_package_XL320" :depends-on ("_package"))
    (:file "XM" :depends-on ("_package_XM"))
    (:file "_package_XM" :depends-on ("_package"))
    (:file "XMExt" :depends-on ("_package_XMExt"))
    (:file "_package_XMExt" :depends-on ("_package"))
  ))