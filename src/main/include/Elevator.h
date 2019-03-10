/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Robot.h"
#include "ElevatorPresets.h"

class Elevator {
 public:

  Elevator(Robot * pRobot);
  //Functions
  bool ElevatorControl(DriverCommands::ElevatorHeight Height, DriverCommands::ElevatorMode Mode,int Offset); // returns whether the elevator is in position
  void WristControl(DriverCommands::ElevatorHeight Height, DriverCommands::ElevatorMode Mode);
  float EncoderTest();

  int IntakeOuttake();
  void RollerIn();
  void RollerOut();
  void RollerPistons(bool bHatchOut);

  void ElevatorTilt(bool Position);// false if forward true if pulled back
  
  void ElevatorControls();

  //Created pointers
  frc::DoubleSolenoid   * EleTilt;
  frc::DoubleSolenoid   * rollerPiston;

  WPI_TalonSRX          LeftRoller{CAN_TALON_LEFT_ROLLER};
  WPI_TalonSRX          RightRoller{CAN_TALON_RIGHT_ROLLER};
  WPI_TalonSRX          ElevatorUpDown{CAN_TALON_ELEVB};
  WPI_TalonSRX          ElevatorUpDownB{CAN_TALON_ELEVA}; 
  WPI_TalonSRX          Wrist{CAN_TALON_WRIST};
  Robot * pRobot;

  //Variables
  double speed = frc::SmartDashboard::GetNumber("Roller Speed", .5);
  DriverCommands::ElevatorHeight    CMDHeight;
  DriverCommands::ElevatorMode      CMDMode;


};
