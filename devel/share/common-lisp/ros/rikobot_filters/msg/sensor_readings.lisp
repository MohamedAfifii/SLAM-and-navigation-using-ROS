; Auto-generated. Do not edit!


(cl:in-package rikobot_filters-msg)


;//! \htmlinclude sensor_readings.msg.html

(cl:defclass <sensor_readings> (roslisp-msg-protocol:ros-message)
  ((imu_msg
    :reader imu_msg
    :initarg :imu_msg
    :type geometry_msgs-msg:Accel
    :initform (cl:make-instance 'geometry_msgs-msg:Accel))
   (odom_msg
    :reader odom_msg
    :initarg :odom_msg
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass sensor_readings (<sensor_readings>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sensor_readings>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sensor_readings)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rikobot_filters-msg:<sensor_readings> is deprecated: use rikobot_filters-msg:sensor_readings instead.")))

(cl:ensure-generic-function 'imu_msg-val :lambda-list '(m))
(cl:defmethod imu_msg-val ((m <sensor_readings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rikobot_filters-msg:imu_msg-val is deprecated.  Use rikobot_filters-msg:imu_msg instead.")
  (imu_msg m))

(cl:ensure-generic-function 'odom_msg-val :lambda-list '(m))
(cl:defmethod odom_msg-val ((m <sensor_readings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rikobot_filters-msg:odom_msg-val is deprecated.  Use rikobot_filters-msg:odom_msg instead.")
  (odom_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sensor_readings>) ostream)
  "Serializes a message object of type '<sensor_readings>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'imu_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'odom_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sensor_readings>) istream)
  "Deserializes a message object of type '<sensor_readings>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'imu_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'odom_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sensor_readings>)))
  "Returns string type for a message object of type '<sensor_readings>"
  "rikobot_filters/sensor_readings")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sensor_readings)))
  "Returns string type for a message object of type 'sensor_readings"
  "rikobot_filters/sensor_readings")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sensor_readings>)))
  "Returns md5sum for a message object of type '<sensor_readings>"
  "97f12acf8e150b22ecdcf2d2e3b6aecb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sensor_readings)))
  "Returns md5sum for a message object of type 'sensor_readings"
  "97f12acf8e150b22ecdcf2d2e3b6aecb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sensor_readings>)))
  "Returns full string definition for message of type '<sensor_readings>"
  (cl:format cl:nil "geometry_msgs/Accel imu_msg~%geometry_msgs/Vector3 odom_msg~%~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sensor_readings)))
  "Returns full string definition for message of type 'sensor_readings"
  (cl:format cl:nil "geometry_msgs/Accel imu_msg~%geometry_msgs/Vector3 odom_msg~%~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sensor_readings>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'imu_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'odom_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sensor_readings>))
  "Converts a ROS message object to a list"
  (cl:list 'sensor_readings
    (cl:cons ':imu_msg (imu_msg msg))
    (cl:cons ':odom_msg (odom_msg msg))
))
