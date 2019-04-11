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

  //  bool drivemode;
  int      stopHoldCounter = 0;
  int      State = 0;
  bool     bRotatePrevious = false;
  bool     EncoderReset = false; // true if encoder has been reset since starting to use the encoder


  //Functions
  void Init();
  void NormalDrive(float * fForward, float * fStrafe, float *fRotate);
  void GyroDrive(float * fForward, float * fStrafe, float *fRotate);
  void FieldOrientedDrive(float * fForward, float * fStrafe, float *fRotate, bool *bFOD);
  void RealVision(float * fForward, float * fStrafe, float *fRotate);
  void GyroTurningDrive();
  void DriveToTarget();
  void Drive(float fForward, float fStrafe, float fRotate, bool bFOD);
  float LimitFWDDrive(float CommandDistance);
  bool DriveToDistance(float CommandDistance, float * fForwarde);
  bool SideUltra(float distance);
  float FindClose(float Angle);

  float EncoderTest();

  void EncoderDrive(bool DriverOne);
};
