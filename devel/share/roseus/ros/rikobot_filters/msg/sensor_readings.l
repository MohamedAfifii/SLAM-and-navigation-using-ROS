;; Auto-generated. Do not edit!


(when (boundp 'rikobot_filters::sensor_readings)
  (if (not (find-package "RIKOBOT_FILTERS"))
    (make-package "RIKOBOT_FILTERS"))
  (shadow 'sensor_readings (find-package "RIKOBOT_FILTERS")))
(unless (find-package "RIKOBOT_FILTERS::SENSOR_READINGS")
  (make-package "RIKOBOT_FILTERS::SENSOR_READINGS"))

(in-package "ROS")
;;//! \htmlinclude sensor_readings.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass rikobot_filters::sensor_readings
  :super ros::object
  :slots (_imu_msg _odom_msg ))

(defmethod rikobot_filters::sensor_readings
  (:init
   (&key
    ((:imu_msg __imu_msg) (instance geometry_msgs::Accel :init))
    ((:odom_msg __odom_msg) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _imu_msg __imu_msg)
   (setq _odom_msg __odom_msg)
   self)
  (:imu_msg
   (&rest __imu_msg)
   (if (keywordp (car __imu_msg))
       (send* _imu_msg __imu_msg)
     (progn
       (if __imu_msg (setq _imu_msg (car __imu_msg)))
       _imu_msg)))
  (:odom_msg
   (&rest __odom_msg)
   (if (keywordp (car __odom_msg))
       (send* _odom_msg __odom_msg)
     (progn
       (if __odom_msg (setq _odom_msg (car __odom_msg)))
       _odom_msg)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Accel _imu_msg
    (send _imu_msg :serialization-length)
    ;; geometry_msgs/Vector3 _odom_msg
    (send _odom_msg :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Accel _imu_msg
       (send _imu_msg :serialize s)
     ;; geometry_msgs/Vector3 _odom_msg
       (send _odom_msg :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Accel _imu_msg
     (send _imu_msg :deserialize buf ptr-) (incf ptr- (send _imu_msg :serialization-length))
   ;; geometry_msgs/Vector3 _odom_msg
     (send _odom_msg :deserialize buf ptr-) (incf ptr- (send _odom_msg :serialization-length))
   ;;
   self)
  )

(setf (get rikobot_filters::sensor_readings :md5sum-) "97f12acf8e150b22ecdcf2d2e3b6aecb")
(setf (get rikobot_filters::sensor_readings :datatype-) "rikobot_filters/sensor_readings")
(setf (get rikobot_filters::sensor_readings :definition-)
      "geometry_msgs/Accel imu_msg
geometry_msgs/Vector3 odom_msg

================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :rikobot_filters/sensor_readings "97f12acf8e150b22ecdcf2d2e3b6aecb")


