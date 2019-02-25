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
  void ElevatorControl(int Offset);
  void WristControl();
  float EncoderTest();

  int IntakeOuttake();
  void RollerIn();
  void RollerOut();
  void RollerLeft();
  void RollerRight();
  void RollerPistons(bool bHatchOut);

  void ElevatorTilt();
  
  void ElevatorControls();

  //Created pointers
  frc::DoubleSolenoid   * EleTilt;
  frc::DoubleSolenoid   * rollerPiston;

  WPI_TalonSRX          LeftRoller{CAN_TALON_LEFT_ROLLER};
  WPI_TalonSRX          RightRoller{CAN_TALON_RIGHT_ROLLER};
  WPI_TalonSRX          ElevatorUpDown{CAN_TALON_ELEV}; 
  WPI_TalonSRX          Wrist{CAN_TALON_WRIST};
  Robot * pRobot;

  //Variables
  double speed = frc::SmartDashboard::GetNumber("Roller Speed", 1);
  DriverCommands::ElevatorHeight    CMDHeight;
  DriverCommands::ElevatorMode      CMDMode;


};
