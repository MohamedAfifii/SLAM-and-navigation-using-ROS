; Auto-generated. Do not edit!


(cl:in-package rikobot_filters-msg)


;//! \htmlinclude sensor_readings.msg.html

(cl:defclass <sensor_readings> (roslisp-msg-protocol:ros-message)
  ((linear
    :reader linear
    :initarg :linear
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (angular
    :reader angular
    :initarg :angular
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (wheel
    :reader wheel
    :initarg :wheel
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass sensor_readings (<sensor_readings>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sensor_readings>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sensor_readings)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rikobot_filters-msg:<sensor_readings> is deprecated: use rikobot_filters-msg:sensor_readings instead.")))

(cl:ensure-generic-function 'linear-val :lambda-list '(m))
(cl:defmethod linear-val ((m <sensor_readings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rikobot_filters-msg:linear-val is deprecated.  Use rikobot_filters-msg:linear instead.")
  (linear m))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <sensor_readings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rikobot_filters-msg:angular-val is deprecated.  Use rikobot_filters-msg:angular instead.")
  (angular m))

(cl:ensure-generic-function 'wheel-val :lambda-list '(m))
(cl:defmethod wheel-val ((m <sensor_readings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rikobot_filters-msg:wheel-val is deprecated.  Use rikobot_filters-msg:wheel instead.")
  (wheel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sensor_readings>) ostream)
  "Serializes a message object of type '<sensor_readings>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'linear) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angular) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wheel) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sensor_readings>) istream)
  "Deserializes a message object of type '<sensor_readings>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'linear) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angular) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wheel) istream)
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
  "20ab05332d8a6eb3abd384abca190819")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sensor_readings)))
  "Returns md5sum for a message object of type 'sensor_readings"
  "20ab05332d8a6eb3abd384abca190819")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sensor_readings>)))
  "Returns full string definition for message of type '<sensor_readings>"
  (cl:format cl:nil "geometry_msgs/Vector3 linear~%geometry_msgs/Vector3 angular~%geometry_msgs/Vector3 wheel~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sensor_readings)))
  "Returns full string definition for message of type 'sensor_readings"
  (cl:format cl:nil "geometry_msgs/Vector3 linear~%geometry_msgs/Vector3 angular~%geometry_msgs/Vector3 wheel~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sensor_readings>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'linear))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angular))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wheel))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sensor_readings>))
  "Converts a ROS message object to a list"
  (cl:list 'sensor_readings
    (cl:cons ':linear (linear msg))
    (cl:cons ':angular (angular msg))
    (cl:cons ':wheel (wheel msg))
))
