
(cl:in-package :asdf)

(defsystem "snazzy_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ProcessCloud" :depends-on ("_package_ProcessCloud"))
    (:file "_package_ProcessCloud" :depends-on ("_package"))
  ))