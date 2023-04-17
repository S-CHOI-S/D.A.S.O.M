// Auto-generated. Do not edit!

// (in-package dynamixel_workbench_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DynamixelState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.model_name = null;
      this.id = null;
      this.torque_enable = null;
      this.goal_current = null;
      this.goal_velocity = null;
      this.goal_position = null;
      this.present_current = null;
      this.present_velocity = null;
      this.present_position = null;
      this.moving = null;
    }
    else {
      if (initObj.hasOwnProperty('model_name')) {
        this.model_name = initObj.model_name
      }
      else {
        this.model_name = '';
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('torque_enable')) {
        this.torque_enable = initObj.torque_enable
      }
      else {
        this.torque_enable = 0;
      }
      if (initObj.hasOwnProperty('goal_current')) {
        this.goal_current = initObj.goal_current
      }
      else {
        this.goal_current = 0;
      }
      if (initObj.hasOwnProperty('goal_velocity')) {
        this.goal_velocity = initObj.goal_velocity
      }
      else {
        this.goal_velocity = 0;
      }
      if (initObj.hasOwnProperty('goal_position')) {
        this.goal_position = initObj.goal_position
      }
      else {
        this.goal_position = 0;
      }
      if (initObj.hasOwnProperty('present_current')) {
        this.present_current = initObj.present_current
      }
      else {
        this.present_current = 0;
      }
      if (initObj.hasOwnProperty('present_velocity')) {
        this.present_velocity = initObj.present_velocity
      }
      else {
        this.present_velocity = 0;
      }
      if (initObj.hasOwnProperty('present_position')) {
        this.present_position = initObj.present_position
      }
      else {
        this.present_position = 0;
      }
      if (initObj.hasOwnProperty('moving')) {
        this.moving = initObj.moving
      }
      else {
        this.moving = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DynamixelState
    // Serialize message field [model_name]
    bufferOffset = _serializer.string(obj.model_name, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.uint8(obj.id, buffer, bufferOffset);
    // Serialize message field [torque_enable]
    bufferOffset = _serializer.uint8(obj.torque_enable, buffer, bufferOffset);
    // Serialize message field [goal_current]
    bufferOffset = _serializer.int16(obj.goal_current, buffer, bufferOffset);
    // Serialize message field [goal_velocity]
    bufferOffset = _serializer.int32(obj.goal_velocity, buffer, bufferOffset);
    // Serialize message field [goal_position]
    bufferOffset = _serializer.int32(obj.goal_position, buffer, bufferOffset);
    // Serialize message field [present_current]
    bufferOffset = _serializer.int16(obj.present_current, buffer, bufferOffset);
    // Serialize message field [present_velocity]
    bufferOffset = _serializer.int32(obj.present_velocity, buffer, bufferOffset);
    // Serialize message field [present_position]
    bufferOffset = _serializer.int32(obj.present_position, buffer, bufferOffset);
    // Serialize message field [moving]
    bufferOffset = _serializer.uint8(obj.moving, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DynamixelState
    let len;
    let data = new DynamixelState(null);
    // Deserialize message field [model_name]
    data.model_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [torque_enable]
    data.torque_enable = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [goal_current]
    data.goal_current = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [goal_velocity]
    data.goal_velocity = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [goal_position]
    data.goal_position = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [present_current]
    data.present_current = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [present_velocity]
    data.present_velocity = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [present_position]
    data.present_position = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [moving]
    data.moving = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.model_name.length;
    return length + 27;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dynamixel_workbench_msgs/DynamixelState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '75ef89ec4862f78146303f5792bac9b7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message includes basic data of dynamixel
    
    string model_name
    uint8  id
    uint8  torque_enable
    
    int16  goal_current
    int32  goal_velocity
    int32 goal_position
    
    int16  present_current
    int32  present_velocity
    int32 present_position
    
    uint8 moving
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DynamixelState(null);
    if (msg.model_name !== undefined) {
      resolved.model_name = msg.model_name;
    }
    else {
      resolved.model_name = ''
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.torque_enable !== undefined) {
      resolved.torque_enable = msg.torque_enable;
    }
    else {
      resolved.torque_enable = 0
    }

    if (msg.goal_current !== undefined) {
      resolved.goal_current = msg.goal_current;
    }
    else {
      resolved.goal_current = 0
    }

    if (msg.goal_velocity !== undefined) {
      resolved.goal_velocity = msg.goal_velocity;
    }
    else {
      resolved.goal_velocity = 0
    }

    if (msg.goal_position !== undefined) {
      resolved.goal_position = msg.goal_position;
    }
    else {
      resolved.goal_position = 0
    }

    if (msg.present_current !== undefined) {
      resolved.present_current = msg.present_current;
    }
    else {
      resolved.present_current = 0
    }

    if (msg.present_velocity !== undefined) {
      resolved.present_velocity = msg.present_velocity;
    }
    else {
      resolved.present_velocity = 0
    }

    if (msg.present_position !== undefined) {
      resolved.present_position = msg.present_position;
    }
    else {
      resolved.present_position = 0
    }

    if (msg.moving !== undefined) {
      resolved.moving = msg.moving;
    }
    else {
      resolved.moving = 0
    }

    return resolved;
    }
};

module.exports = DynamixelState;
