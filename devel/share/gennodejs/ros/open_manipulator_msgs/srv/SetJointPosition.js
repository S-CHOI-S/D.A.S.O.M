// Auto-generated. Do not edit!

// (in-package open_manipulator_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let JointPosition = require('../msg/JointPosition.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetJointPositionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_position = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_position')) {
        this.joint_position = initObj.joint_position
      }
      else {
        this.joint_position = new JointPosition();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetJointPositionRequest
    // Serialize message field [joint_position]
    bufferOffset = JointPosition.serialize(obj.joint_position, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetJointPositionRequest
    let len;
    let data = new SetJointPositionRequest(null);
    // Deserialize message field [joint_position]
    data.joint_position = JointPosition.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += JointPosition.getMessageSize(object.joint_position);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'open_manipulator_msgs/SetJointPositionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e1f1ee99b5e77308297dc4eeedd305d4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    JointPosition joint_position
    
    ================================================================================
    MSG: open_manipulator_msgs/JointPosition
    string[]   joint_name
    float64[]  position
    float64    max_accelerations_scaling_factor
    float64    max_velocity_scaling_factor
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetJointPositionRequest(null);
    if (msg.joint_position !== undefined) {
      resolved.joint_position = JointPosition.Resolve(msg.joint_position)
    }
    else {
      resolved.joint_position = new JointPosition()
    }

    return resolved;
    }
};

class SetJointPositionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isPlanned = null;
    }
    else {
      if (initObj.hasOwnProperty('isPlanned')) {
        this.isPlanned = initObj.isPlanned
      }
      else {
        this.isPlanned = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetJointPositionResponse
    // Serialize message field [isPlanned]
    bufferOffset = _serializer.bool(obj.isPlanned, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetJointPositionResponse
    let len;
    let data = new SetJointPositionResponse(null);
    // Deserialize message field [isPlanned]
    data.isPlanned = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'open_manipulator_msgs/SetJointPositionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c4a8e64ceeeccdab98609099e2b0c166';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isPlanned
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetJointPositionResponse(null);
    if (msg.isPlanned !== undefined) {
      resolved.isPlanned = msg.isPlanned;
    }
    else {
      resolved.isPlanned = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SetJointPositionRequest,
  Response: SetJointPositionResponse,
  md5sum() { return '51816c1a47e918af5b30ea571f8202f7'; },
  datatype() { return 'open_manipulator_msgs/SetJointPosition'; }
};
