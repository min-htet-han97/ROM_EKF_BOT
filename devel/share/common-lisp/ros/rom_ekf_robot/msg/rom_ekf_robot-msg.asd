
(cl:in-package :asdf)

(defsystem "rom_ekf_robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MapData" :depends-on ("_package_MapData"))
    (:file "_package_MapData" :depends-on ("_package"))
  ))