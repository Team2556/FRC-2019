/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

class RobotTeleop {
 public:
  // Contructor
  RobotTeleop(Robot * pRobot);

  // Members
  Robot * pRobot;

  // Methods
  void Init();
  void Periodic();
};
