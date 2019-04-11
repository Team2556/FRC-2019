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
  void Auto1(); // left front rocket hatch
  void Auto1Init();
  void Auto2(); // left back rocket hatch
  void Auto2Init();
  void Auto3(); // left cargoship cargo close w/o encoder
  void Auto3Init();
  void Auto4(); // left cargoship cargo w/ encoder
  void Auto4Init();
  void Auto5();
  void Auto5Init();
  bool GetOffHab(float *fForward, float *fStrafe, bool *bFOD);
  bool ZeroElevator();

  void EncoderDrive(bool ControllerOne);
  bool DriveToEncoder(int Rotation);

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
