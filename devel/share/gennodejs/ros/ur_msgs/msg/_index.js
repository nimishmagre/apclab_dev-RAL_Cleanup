
"use strict";

let MasterboardDataMsg = require('./MasterboardDataMsg.js');
let ToolDataMsg = require('./ToolDataMsg.js');
let RobotStateRTMsg = require('./RobotStateRTMsg.js');
let Analog = require('./Analog.js');
let IOStates = require('./IOStates.js');
let Digital = require('./Digital.js');

module.exports = {
  MasterboardDataMsg: MasterboardDataMsg,
  ToolDataMsg: ToolDataMsg,
  RobotStateRTMsg: RobotStateRTMsg,
  Analog: Analog,
  IOStates: IOStates,
  Digital: Digital,
};
