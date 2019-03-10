/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#pragma once

#include "frc/WPILib.h"

#include "robotmap.h"
#include "NavGyro.h"

#define LT_SENSORS_FRONT  6
#define LT_SENSORS_BACK   2

class ColorSens {
 public:
  ColorSens();

  void    UpdateValues();
  void    UpdateBackValues();
  double  GetStrafe(float InitStrafe);
  double  GetRotate(float InitRotate, bool AllowRotate);
  
  double Strafes[LT_SENSORS_FRONT] = {1.0, 0.66, 0.33, -0.33, -0.66, -1.0};
  double Rotates[LT_SENSORS_BACK]  = {-0.1, 0.1};

  // Map sensor DIO inputs to sensor array
  frc::DigitalInput  RLSens[LT_SENSORS_FRONT] = 
  {
    frc::DigitalInput(DIO_LT_FRONT_1),
    frc::DigitalInput(DIO_LT_FRONT_2),
    frc::DigitalInput(DIO_LT_FRONT_3),
    frc::DigitalInput(DIO_LT_FRONT_4),
    frc::DigitalInput(DIO_LT_FRONT_5),
    frc::DigitalInput(DIO_LT_FRONT_6)
  };

  frc::DigitalInput BackSens[LT_SENSORS_BACK] = 
  {
    frc::DigitalInput(DIO_LT_BACK_1),
    frc::DigitalInput(DIO_LT_BACK_2)
  };

  // Structures to hold front and back sensor info
  struct
  {
    bool    Values[LT_SENSORS_FRONT];
    bool    bLineFound;
    int     iMinIndex;
    int     iMaxIndex;
  } FrontSensors;

  struct
  {
    bool    Values[LT_SENSORS_BACK];
    bool    bLineFound;
    int     iMinIndex;
    int     iMaxIndex;
  } BackSensors;

};
