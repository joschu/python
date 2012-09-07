
(cl:in-package :asdf)

(defsystem "snazzy_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CropBox" :depends-on ("_package_CropBox"))
    (:file "_package_CropBox" :depends-on ("_package"))
  ))