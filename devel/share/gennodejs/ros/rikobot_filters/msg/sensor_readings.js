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
      this.linear = null;
      this.angular = null;
      this.wheel = null;
    }
    else {
      if (initObj.hasOwnProperty('linear')) {
        this.linear = initObj.linear
      }
      else {
        this.linear = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('angular')) {
        this.angular = initObj.angular
      }
      else {
        this.angular = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('wheel')) {
        this.wheel = initObj.wheel
      }
      else {
        this.wheel = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sensor_readings
    // Serialize message field [linear]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.linear, buffer, bufferOffset);
    // Serialize message field [angular]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.angular, buffer, bufferOffset);
    // Serialize message field [wheel]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.wheel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sensor_readings
    let len;
    let data = new sensor_readings(null);
    // Deserialize message field [linear]
    data.linear = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [angular]
    data.angular = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [wheel]
    data.wheel = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
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
    return '20ab05332d8a6eb3abd384abca190819';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Vector3 linear
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sensor_readings(null);
    if (msg.linear !== undefined) {
      resolved.linear = geometry_msgs.msg.Vector3.Resolve(msg.linear)
    }
    else {
      resolved.linear = new geometry_msgs.msg.Vector3()
    }

    if (msg.angular !== undefined) {
      resolved.angular = geometry_msgs.msg.Vector3.Resolve(msg.angular)
    }
    else {
      resolved.angular = new geometry_msgs.msg.Vector3()
    }

    if (msg.wheel !== undefined) {
      resolved.wheel = geometry_msgs.msg.Vector3.Resolve(msg.wheel)
    }
    else {
      resolved.wheel = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = sensor_readings;
