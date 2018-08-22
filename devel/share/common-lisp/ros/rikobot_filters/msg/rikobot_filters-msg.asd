
(cl:in-package :asdf)

(defsystem "rikobot_filters-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "sensor_readings" :depends-on ("_package_sensor_readings"))
    (:file "_package_sensor_readings" :depends-on ("_package"))
  ))