/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#pragma once
#include "frc/WPILib.h"
#include "NavGyro.h"


class ColorSens {
 public:
  ColorSens();

  bool    LineFound();
  bool    BackLineFound();
  void    UpdateValues();
  void    UpdateBackValues();
  double  GetStrafe(float InitStrafe, bool AllowStrafe);
  double  GetRotate(float InitRotate, bool AllowRotate);
  void    FindMinMax(int * Min, int * Max);
  void    BackMinMax(int * Min, int * Max);
  void    SetRotate(NavGyro * pNavGyro, bool AllowRotate);
  float   FindClose(float Angle);


  double Angles[8] = {61.25, 118.75, 90, -90, -118.75, -61.25, 180, 0};
  double Strafes[6] = {1,.66,.33,-.33,-.66,-1};
  double Rotates[2] = {-.1,.1};
  bool GyroReset;
  
  frc::DigitalInput  RLSens[6] = 
  {
    frc::DigitalInput(0),
    frc::DigitalInput(1),
    frc::DigitalInput(2),
    frc::DigitalInput(3),
    frc::DigitalInput(4),
    frc::DigitalInput(5)
  };

  frc::DigitalInput BackSens[2] = 
  {
    frc::DigitalInput(6),
    frc::DigitalInput(7)
  };
  
  bool                Values[6];
};
