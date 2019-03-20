/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/wpilib.h"
#include "Robot.h"

class Climb {
 public:
  // Constructor
  Climb(Robot * pRobot);

  // Variables
  Robot                 * pRobot;
  frc::DoubleSolenoid   * FrontClimb;
  frc::DoubleSolenoid   * RearClimb;
  int                     isClimbing = 0;
  float                   fInitPitch = 0;
  int                     ClimbCounter = 0;
  float                   initBadTilt = 0.0;
  int                     Timer = 0;

  // Methods
  void Climbing();
  void Oscillation(int Side);// 0 is the front and 1 is the back
  void test();
  void ShuffleForward();
  void HoldIn();

};
