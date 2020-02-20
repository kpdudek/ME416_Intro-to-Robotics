
(cl:in-package :asdf)

(defsystem "me416_lab-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MotorSpeeds" :depends-on ("_package_MotorSpeeds"))
    (:file "_package_MotorSpeeds" :depends-on ("_package"))
    (:file "MotorSpeedsStamped" :depends-on ("_package_MotorSpeedsStamped"))
    (:file "_package_MotorSpeedsStamped" :depends-on ("_package"))
  ))