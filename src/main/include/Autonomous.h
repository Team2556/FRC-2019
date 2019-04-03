/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//#define AUTO_ELE_ENABLED

#pragma once
#include "Robot.h"
#include "DriveBase.h"
#include "Elevator.h"
#include "TeleopControl.h"
#include "DriverCommands.h"

class Autonomous {
 public:
  Autonomous(Robot * pRobot, DriveBase * MecDrive, Elevator * ControlElevator, TeleopControl * TeleopAuto);

  void Auto();
  void AutoInit();

  // Auto Modes
  void AutoTeleop();
  void AutoTeleopInit();
  void Auto1();
  void Auto1Init();
  void Auto2();
  void Auto2Init();
  bool RollersSet();

  Robot * pRobot;
  DriveBase * MecDrive;
  Elevator * ControlElevator;
  TeleopControl * TeleopAuto;


  int     AutoCounter = 0;
  int     SectionStart = 0;
  int     AutoNumber;
  int     ActionNum;
  bool    FieldOrientedDrive = false;
};
