/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

// FRC includes
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "NavGyro.h"

class Robot : public frc::TimedRobot {
 public:

  frc::XboxController   Xbox1{0};
  frc::XboxController   Xbox2{1};
  WPI_TalonSRX          MotorControl_LF{1};
  WPI_TalonSRX          MotorControl_RF{2};
  WPI_TalonSRX          MotorControl_LR{3};
  WPI_TalonSRX          MotorControl_RR{4};
	frc::MecanumDrive     RobotDrive{MotorControl_LF, MotorControl_LR, MotorControl_RF, MotorControl_RR};
  NavGyro               pNavGyro{};
  cs::UsbCamera		      UsbCamera1;
  

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
