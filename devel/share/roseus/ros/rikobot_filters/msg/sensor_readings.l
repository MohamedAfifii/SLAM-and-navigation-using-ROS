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
  :slots (_linear _angular _wheel ))

(defmethod rikobot_filters::sensor_readings
  (:init
   (&key
    ((:linear __linear) (instance geometry_msgs::Vector3 :init))
    ((:angular __angular) (instance geometry_msgs::Vector3 :init))
    ((:wheel __wheel) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _linear __linear)
   (setq _angular __angular)
   (setq _wheel __wheel)
   self)
  (:linear
   (&rest __linear)
   (if (keywordp (car __linear))
       (send* _linear __linear)
     (progn
       (if __linear (setq _linear (car __linear)))
       _linear)))
  (:angular
   (&rest __angular)
   (if (keywordp (car __angular))
       (send* _angular __angular)
     (progn
       (if __angular (setq _angular (car __angular)))
       _angular)))
  (:wheel
   (&rest __wheel)
   (if (keywordp (car __wheel))
       (send* _wheel __wheel)
     (progn
       (if __wheel (setq _wheel (car __wheel)))
       _wheel)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _linear
    (send _linear :serialization-length)
    ;; geometry_msgs/Vector3 _angular
    (send _angular :serialization-length)
    ;; geometry_msgs/Vector3 _wheel
    (send _wheel :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _linear
       (send _linear :serialize s)
     ;; geometry_msgs/Vector3 _angular
       (send _angular :serialize s)
     ;; geometry_msgs/Vector3 _wheel
       (send _wheel :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _linear
     (send _linear :deserialize buf ptr-) (incf ptr- (send _linear :serialization-length))
   ;; geometry_msgs/Vector3 _angular
     (send _angular :deserialize buf ptr-) (incf ptr- (send _angular :serialization-length))
   ;; geometry_msgs/Vector3 _wheel
     (send _wheel :deserialize buf ptr-) (incf ptr- (send _wheel :serialization-length))
   ;;
   self)
  )

(setf (get rikobot_filters::sensor_readings :md5sum-) "20ab05332d8a6eb3abd384abca190819")
(setf (get rikobot_filters::sensor_readings :datatype-) "rikobot_filters/sensor_readings")
(setf (get rikobot_filters::sensor_readings :definition-)
      "geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
geometry_msgs/Vector3 wheel

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



(provide :rikobot_filters/sensor_readings "20ab05332d8a6eb3abd384abca190819")


