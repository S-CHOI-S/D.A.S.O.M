// Auto-generated. Do not edit!

// (in-package open_manipulator_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let KinematicsPose = require('../msg/KinematicsPose.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetKinematicsPoseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.kinematics_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('kinematics_pose')) {
        this.kinematics_pose = initObj.kinematics_pose
      }
      else {
        this.kinematics_pose = new KinematicsPose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetKinematicsPoseRequest
    // Serialize message field [kinematics_pose]
    bufferOffset = KinematicsPose.serialize(obj.kinematics_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetKinematicsPoseRequest
    let len;
    let data = new SetKinematicsPoseRequest(null);
    // Deserialize message field [kinematics_pose]
    data.kinematics_pose = KinematicsPose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += KinematicsPose.getMessageSize(object.kinematics_pose);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'open_manipulator_msgs/SetKinematicsPoseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '061ceb25b20ec55ef37bf7bab518c5c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    KinematicsPose kinematics_pose
    
    ================================================================================
    MSG: open_manipulator_msgs/KinematicsPose
    string    	        group_name
    geometry_msgs/Pose  pose
    float64    max_accelerations_scaling_factor
    float64    max_velocity_scaling_factor
    float64    tolerance
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetKinematicsPoseRequest(null);
    if (msg.kinematics_pose !== undefined) {
      resolved.kinematics_pose = KinematicsPose.Resolve(msg.kinematics_pose)
    }
    else {
      resolved.kinematics_pose = new KinematicsPose()
    }

    return resolved;
    }
};

class SetKinematicsPoseResponse {
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
    // Serializes a message object of type SetKinematicsPoseResponse
    // Serialize message field [isPlanned]
    bufferOffset = _serializer.bool(obj.isPlanned, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetKinematicsPoseResponse
    let len;
    let data = new SetKinematicsPoseResponse(null);
    // Deserialize message field [isPlanned]
    data.isPlanned = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'open_manipulator_msgs/SetKinematicsPoseResponse';
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
    const resolved = new SetKinematicsPoseResponse(null);
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
  Request: SetKinematicsPoseRequest,
  Response: SetKinematicsPoseResponse,
  md5sum() { return '4841916847c645ef404d13cebf9595cb'; },
  datatype() { return 'open_manipulator_msgs/SetKinematicsPose'; }
};
