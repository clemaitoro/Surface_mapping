
"use strict";

let SetAnalogOutput = require('./SetAnalogOutput.js')
let SetSpeedSliderFraction = require('./SetSpeedSliderFraction.js')
let SetIO = require('./SetIO.js')
let SetPayload = require('./SetPayload.js')
let GetRobotSoftwareVersion = require('./GetRobotSoftwareVersion.js')

module.exports = {
  SetAnalogOutput: SetAnalogOutput,
  SetSpeedSliderFraction: SetSpeedSliderFraction,
  SetIO: SetIO,
  SetPayload: SetPayload,
  GetRobotSoftwareVersion: GetRobotSoftwareVersion,
};
