/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Robot.h"
#include "DriveBase.h"
#include "Elevator.h"
#include "Climb.h"

class TeleopControl 
{
 public:
  TeleopControl(Robot * pRobot, DriveBase * MecDrive, Elevator * ControlElevator, Climb * Climber);



  Robot       * pRobot;
  DriveBase   * MecDrive;
  Elevator    * ControlElevator;
  Climb       * Climber;

  void TeleopMain();
  void TeleopDrive();
  void TeleopElevator();
  void AutoLineUp(float * fForward, float * fStrafe, float *fRotate);
  



  // Variables
  int AutoLineUpState = 0;
};
