/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Robot.h"
#include "DriveBase.h"

class Autonomous {
 public:
  Autonomous(Robot * pRobot, DriveBase * MecDrive);

  void Auto();

  // Auto Modes
  void AutoTeleop();
  void AutoTeleopInit();
  void Auto1();
  void Auto1Init();
  void Auto2();
  void Auto2Init();

  Robot * pRobot;
  DriveBase * MecDrive;
  int     AutoCounter = 0;
  int     SectionStart = 0;
  int     AutoNumber;
  int     ActionNum;
  bool    FieldOrientedDrive = false;
};
