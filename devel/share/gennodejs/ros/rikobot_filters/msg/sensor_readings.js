// Auto-generated. Do not edit!

// (in-package rikobot_filters.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class sensor_readings {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.imu_msg = null;
      this.odom_msg = null;
    }
    else {
      if (initObj.hasOwnProperty('imu_msg')) {
        this.imu_msg = initObj.imu_msg
      }
      else {
        this.imu_msg = new geometry_msgs.msg.Accel();
      }
      if (initObj.hasOwnProperty('odom_msg')) {
        this.odom_msg = initObj.odom_msg
      }
      else {
        this.odom_msg = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sensor_readings
    // Serialize message field [imu_msg]
    bufferOffset = geometry_msgs.msg.Accel.serialize(obj.imu_msg, buffer, bufferOffset);
    // Serialize message field [odom_msg]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.odom_msg, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sensor_readings
    let len;
    let data = new sensor_readings(null);
    // Deserialize message field [imu_msg]
    data.imu_msg = geometry_msgs.msg.Accel.deserialize(buffer, bufferOffset);
    // Deserialize message field [odom_msg]
    data.odom_msg = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rikobot_filters/sensor_readings';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '97f12acf8e150b22ecdcf2d2e3b6aecb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Accel imu_msg
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sensor_readings(null);
    if (msg.imu_msg !== undefined) {
      resolved.imu_msg = geometry_msgs.msg.Accel.Resolve(msg.imu_msg)
    }
    else {
      resolved.imu_msg = new geometry_msgs.msg.Accel()
    }

    if (msg.odom_msg !== undefined) {
      resolved.odom_msg = geometry_msgs.msg.Vector3.Resolve(msg.odom_msg)
    }
    else {
      resolved.odom_msg = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = sensor_readings;
