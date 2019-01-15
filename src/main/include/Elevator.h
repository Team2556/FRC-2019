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
  //Function
    void Output(void);

  //Created pointers//
    frc::DoubleSolenoid * hatchSolenoid;
    Robot * pRobot;

};
