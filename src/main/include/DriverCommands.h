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
  enum ElevatorHeight {Low, Middle, High, Pickup, GroundPickup, CargoShip};
  int  iElevatorHeight = 0; // 0 = low, 1 = middle, 2 = high, 3 = Cargo Ship -1 is pickup, -2 is ground pickup
  enum ElevatorMode {Hatch, Cargo};
  
  DriveMode           CurrDriveMode;
  ElevatorHeight      CMDElevatorHeight;
  ElevatorMode        CMDElevatorMode;

  bool        RollersDown = false; // true if the rollers/wrist are down
  bool        ElevatorTilted = false; // true if the elevator is tilted backwards
  bool        rollerBool = false;//true if roller pistons pushed out
  bool        ClimbMode  = false; // false normally, true when climbing
  bool        PrevReset = false; // true if prev reset was facing forward
  bool        CurrentlyClimbing = false; // true while we are lifting, false normally

  // General Commands
  bool      GetClimbMode();

  // Moving commands
  float     fMoveForward();
  float     fMoveSideways();
  float     fRotate();
  bool      bManualRotate();
  bool      bResetGyro();
  int       POV();
  DriveMode GetDriveMode();
  bool      GetLineUp();
  bool      UltrasonicAllowed();
  double    GetAutoStrafe();

  // Elevator commands
  float           fElevatorUpDownSpeed();
  bool            bRollersDown();
  bool            bRollerPistons();
  bool            bElevatorTilt();
  ElevatorMode    GetElevatorMode();
  ElevatorHeight  GetElevatorHeight();
  void            HeightIntEnum();// sets the elevator height enum based off the height int 
  bool            Intake();
  bool            Outtake();
  bool            bAutomaticElevator();

  // Climber commands

  bool            bCurrentlyClimbing();// determines whether we are actively raising the robot to climnb
  // Test commands
  bool      bTestButton(int iButton);
  float     fTestValue(int iControl);
};
