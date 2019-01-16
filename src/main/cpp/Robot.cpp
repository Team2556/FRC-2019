/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "DriveBase.h"
#include "Elevator.h"


// Objects and variable for this file only
DriveBase         * MecDrive;
Elevator          * ControlElevator;


#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {

  MecDrive          = new DriveBase(this);
  ControlElevator   = new Elevator(this);
  
  pNavGyro.Init();
  }

// ----------------------------------------------------------------------------


void Robot::RobotPeriodic() 
{

}

void Robot::AutonomousInit() 
{
 
}

void Robot::AutonomousPeriodic() 
{
}

void Robot::TeleopInit() 
{

}

// ----------------------------------------------------------------------------

void Robot::TeleopPeriodic() 
  {
  //Teleop Functions
    MecDrive->Drive();
    ControlElevator->Output();
  }

// ============================================================================

void Robot::TestPeriodic() 
{

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
