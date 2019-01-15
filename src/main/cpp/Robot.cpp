/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "RobotPeriodic.h"
#include "RobotAutonomous.h"
#include "RobotTeleop.h"
#include "DriveBase.h"
#include "Elevator.h"


// Objects and variable for this file only
RobotTeleop       * ControlTeleop;
RobotAutonomous   * ControlAutonomous;
DriveBase         * MecDrive;
Elevator          * ControlElevator;


#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  MecDrive          = new DriveBase(this);
  ControlTeleop     = new RobotTeleop(this);
  ControlAutonomous = new RobotAutonomous(this);
  ControlElevator   = new Elevator(this);
  
  pNavGyro.Init();
  }

// ----------------------------------------------------------------------------

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
 
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {}

// ----------------------------------------------------------------------------

void Robot::TeleopPeriodic() 
  {
    
    MecDrive->Drive();
    ControlTeleop->Periodic();  
    ControlElevator->Output();
  }

// ============================================================================

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
