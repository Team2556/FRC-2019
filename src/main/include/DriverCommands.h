/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/wpilib.h"

#include "RobotMap.h"

//#define JOYSTICK

class DriverCommands 
{
public:
  DriverCommands();

protected:
#ifdef JOYSTICK
  frc::Joystick         JStick1{0};
#endif
  frc::XboxController   Xbox1{XBOX_ONE};
  frc::XboxController   Xbox2{XBOX_TWO};

public:

  // Variables
  enum DriveMode {Unknown, Normal, Gyro, FieldOriented, DriveToTarget};
  
  DriveMode   CurrDriveMode;

  // Moving commands
  float     fMoveForward();
  float     fMoveSideways();
  float     fRotate();
  bool      bManualRotate();
  bool      bResetGyro();
  int       POV();
  DriveMode GetDriveMode();

  // Elevator commands
  float     fElevatorUpDownSpeed();
  bool      bRollersDown();

  // Climber commands


  // Test commands
  bool      bTestButton(int iButton);
  float     fTestValue(int iControl);
};
