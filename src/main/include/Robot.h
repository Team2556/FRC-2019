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

class Robot : public frc::TimedRobot {
 public:

/*<<<<<<< HEAD
  frc::XboxController   Xbox1{0};
  frc::XboxController   Xbox2{1};
  WPI_TalonSRX          MotorControl_LF{1};
  WPI_TalonSRX          MotorControl_RF{2};
  WPI_TalonSRX          MotorControl_LR{3};
  WPI_TalonSRX          MotorControl_RR{4};
  WPI_TalonSRX          Elevator_Motor{8};
  WPI_TalonSRX          Intake1{6};
  WPI_TalonSRX          Intake2{7}; 
S======= */
  frc::XboxController   Xbox1{XBOX_ONE};
  frc::XboxController   Xbox2{XBOX_TWO};
  WPI_TalonSRX          MotorControl_LF{CAN_TALON_LF};
  WPI_TalonSRX          MotorControl_RF{CAN_TALON_RF};
  WPI_TalonSRX          MotorControl_LR{CAN_TALON_LR};
  WPI_TalonSRX          MotorControl_RR{CAN_TALON_RR};
//>>>>>>> f36a2818c8ab540fe32be153d75c1d8c3a6f0e55
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
