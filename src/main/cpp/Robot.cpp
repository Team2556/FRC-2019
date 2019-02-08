/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <thread>         // std::thread

#include "frc/WPILib.h"

#include "Robot.h"
#include "DriveBase.h"
#include "Elevator.h"
#include "Climb.h"
#include "Autonomous.h"

// Objects and variable for this file only
DriveBase         * MecDrive;
Elevator          * ControlElevator;
Climb             * Climber;
Autonomous        * Autos;

// ----------------------------------------------------------------------------

void Robot::RobotInit() {
    pPrefs = frc::Preferences::GetInstance();

  MecDrive          = new DriveBase(this);
  ControlElevator   = new Elevator(this);
  Climber           = new Climb(this);
  Autos             = new Autonomous(this, MecDrive);
  
  Nav.Init(false);
  MecDrive->Init();
<<<<<<< HEAD
/*
  UsbCamera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture();
  UsbCamera1.SetResolution(160, 120);
  UsbCamera1.SetFPS(20);
*/
=======

#ifdef USB_CAMERA
  UsbCamera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  UsbCamera1.SetResolution(160, 120);
  UsbCamera1.SetFPS(20);
#endif

#ifdef AXIS_CAMERA
  AxisCamera1 = frc::CameraServer::GetInstance()->AddAxisCamera("11.25.56.17");
  AxisCamera1.SetResolution(320, 240);
  AxisCamera1.SetFPS(10);
#endif

#ifdef CAMERA
  CameraTrk.Init();
  frc::SmartDashboard::PutNumber("Vision Display", CameraTrk.iDisplayFrame);
  pVisionThread = new std::thread(&CameraTrack::TrackThread, &CameraTrk);
#endif

>>>>>>> 42138cbee9d0e8c5beeb698ba600dbf9060b469d
  int timing = frc::SmartDashboard::GetNumber("Timing", 10);
  frc::SmartDashboard::PutNumber("Timing", timing);
  int period = frc::SmartDashboard::GetNumber("Shuffle Period", 1);
  frc::SmartDashboard::PutNumber("Shuffle Period", period);// time betwwen full shuffles
  int delay = frc::SmartDashboard::GetNumber("Switch Delay", 1);
  frc::SmartDashboard::PutNumber("Switch Delay", delay);// delay between raising and droping front pistons
  double speed = frc::SmartDashboard::GetNumber("Roller Speed", 1);
  frc::SmartDashboard::PutNumber("Roller Speed", speed);

  }

// ----------------------------------------------------------------------------

void Robot::RobotPeriodic() 
{
#ifdef CAMERA
    CameraTrk.iDisplayFrame = frc::SmartDashboard::GetNumber("Vision Display", 0);
    pPrefs->PutInt("Display Frame", CameraTrk.iDisplayFrame);
#endif
}

// ----------------------------------------------------------------------------

void Robot::AutonomousInit() 
{
  //Auto selector will go here and run the coresponding Auto's Init to select it
  //For now there is only one auto for testing purposes so its init will allways be called
  Autos->Auto1Init();
}

// ----------------------------------------------------------------------------

void Robot::AutonomousPeriodic() 
{
  LineTracker.UpdateValues();
  LineTracker.UpdateBackValues();
  Autos->Auto1();
}

// ----------------------------------------------------------------------------

void Robot::TeleopInit() 
{
  Nav.SetCommandYawToCurrent();
}

// ----------------------------------------------------------------------------

void Robot::TeleopPeriodic() 
  {
    //Teleop Functions
    MecDrive->Drive();
    LineTracker.UpdateValues();
    ControlElevator->ElevatorControls();
    //Climber->Climbing();
    SmartDashboard::PutNumber("Angle", Nav.GetYaw());
  }

// ============================================================================

void Robot::TestPeriodic() 
{

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
