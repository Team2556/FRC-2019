/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

class Elevator {
 public:

  Elevator(Robot * pRobot);
  //Functions
  void Intake();
  void Output();
  void RollerIn();
  void RollerOut();
  void RollerLeft();
  void RollerRight();
  void CoDriveControls();

  //Created pointers
  frc::DoubleSolenoid * hatchSolenoid;
  WPI_TalonSRX          LeftRoller{1}; // will become 5 when we have enough motor controllers
  WPI_TalonSRX          RightRoller{2};// will become 6 when we have enough motor controllers
  

    Robot * pRobot;

  //Variables
  double speed = SmartDashboard::GetNumber("Roller Speed", 1);

};
