/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Robot.h"

class Elevator {
 public:

  Elevator(Robot * pRobot);
  //Functions
  void ElevatorControl();
  void RollersControl();

  void RollerIn();
  void RollerOut();
  void RollerLeft();
  void RollerRight();
  
  void CoDriveControls();

  //Created pointers
  frc::DoubleSolenoid * hatchSolenoid;
  WPI_TalonSRX          LeftRoller{CAN_TALON_LEFT_ROLLER}; // will become 5 when we have enough motor controllers
  WPI_TalonSRX          RightRoller{CAN_TALON_LEFT_ROLLER};// will become 6 when we have enough motor controllers
  WPI_TalonSRX          ElevatorLeft{CAN_TALON_LEFT_ELEV}; // will become 5 when we have enough motor controllers
  WPI_TalonSRX          ElevatorRight{CAN_TALON_RIGHT_ELEV};// will become 6 when we have enough motor controllers

  Robot * pRobot;

  //Variables
  double speed = frc::SmartDashboard::GetNumber("Roller Speed", 1);

};
