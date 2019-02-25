/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

// FRC includes
#include "RobotMap.h"
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "NavGyro.h"
#include "DriverCommands.h"
#include "CameraTrack.h"
#include "LineTrack.h"


class Robot : public frc::TimedRobot {
 public:

#ifdef USB_CAMERA
  cs::UsbCamera         UsbCamera1;
#endif
#ifdef AXIS_CAMERA
  cs::AxisCamera        AxisCamera1{"AXIS FRONT", "10.25.56.17"};
#endif
#ifdef CAMERA
  frc::CameraServer   * pCamServer;
#endif

  std::thread         * pVisionThread;
  CameraTrack           CameraTrk;
  WPI_TalonSRX          MotorControl_LF{CAN_TALON_LF};
  WPI_TalonSRX          MotorControl_RF{CAN_TALON_RF};
  WPI_TalonSRX          MotorControl_LR{CAN_TALON_LR};
  WPI_TalonSRX          MotorControl_RR{CAN_TALON_RR};
	frc::MecanumDrive     RobotDrive{MotorControl_LF, MotorControl_LR, MotorControl_RF, MotorControl_RR};
  NavGyro               Nav;
  DriverCommands        DriverCmd;
  ColorSens             LineTracker;
  Ultrasonic            UltraLF{DIO_US_RANGE_TRIGGER_LF, DIO_US_RANGE_PULSE_LF, Ultrasonic::DistanceUnit::kInches};
  Ultrasonic            UltraRF{DIO_US_RANGE_TRIGGER_RF, DIO_US_RANGE_PULSE_RF, Ultrasonic::DistanceUnit::kInches};
  
  frc::Preferences   *  pPrefs;
    
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;


 private:
  frc::SendableChooser<std::string> AutoChooser;
  
  const std::string AutoTeleop = "Teleop";
  const std::string Auto1 = "Left Rocket Front";
  const std::string Auto2 = "Left Rocket Back";
  std::string AutoMode;
};
