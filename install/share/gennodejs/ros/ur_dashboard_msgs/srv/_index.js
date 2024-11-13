
"use strict";

let IsProgramRunning = require('./IsProgramRunning.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let Popup = require('./Popup.js')
let AddToLog = require('./AddToLog.js')
let Load = require('./Load.js')
let RawRequest = require('./RawRequest.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let GetProgramState = require('./GetProgramState.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let GetRobotMode = require('./GetRobotMode.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')

module.exports = {
  IsProgramRunning: IsProgramRunning,
  IsInRemoteControl: IsInRemoteControl,
  Popup: Popup,
  AddToLog: AddToLog,
  Load: Load,
  RawRequest: RawRequest,
  IsProgramSaved: IsProgramSaved,
  GetProgramState: GetProgramState,
  GetSafetyMode: GetSafetyMode,
  GetRobotMode: GetRobotMode,
  GetLoadedProgram: GetLoadedProgram,
};
