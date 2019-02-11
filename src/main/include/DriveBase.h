/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Robot.h"

class DriveBase {
 public:
  //constructer
  DriveBase(Robot * pRobot);

  //Members
  Robot * pRobot;
  frc::Ultrasonic   Ultra{DIO_US_RANGE_TRIGGER, DIO_US_RANGE_PULSE,  frc::Ultrasonic::DistanceUnit::kInches};
  frc::Ultrasonic   UltraLeft{DIO_US_RANGE_TRIGGER_LEFT, DIO_US_RANGE_PULSE_LEFT,  frc::Ultrasonic::DistanceUnit::kInches};
//  bool drivemode;
  int      stopHoldCounter = 0;
  bool     bRotatePrevious = false;


  //Functions
  void Init();
  void NormalDrive();
  void GyroDrive();
  void FieldOrientedDrive();
  void OldFieldOrientedDrive();
  void GyroTurningDrive();
  void DriveToTarget();
  void Drive();
  float LimitFWDDrive(float InitDrive, bool Auto, float CommandDistance);
  bool SideUltra(float distance);

  float EncoderTest();
  
};
