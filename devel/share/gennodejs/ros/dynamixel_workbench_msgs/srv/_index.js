
"use strict";

let JointCommand = require('./JointCommand.js')
let test = require('./test.js')
let DynamixelCommand = require('./DynamixelCommand.js')
let WheelCommand = require('./WheelCommand.js')
let EECommand = require('./EECommand.js')
let GetDynamixelInfo = require('./GetDynamixelInfo.js')

module.exports = {
  JointCommand: JointCommand,
  test: test,
  DynamixelCommand: DynamixelCommand,
  WheelCommand: WheelCommand,
  EECommand: EECommand,
  GetDynamixelInfo: GetDynamixelInfo,
};
