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
#include "TeleopControl.h"


// Objects and variable for this file only
DriveBase         * MecDrive;
Elevator          * ControlElevator;
Climb             * Climber;
Autonomous        * Autos;
TeleopControl     * TeleopMain;

// ----------------------------------------------------------------------------

void Robot::RobotInit() {
  pPrefs = frc::Preferences::GetInstance();

  MecDrive          = new DriveBase(this);
  ControlElevator   = new Elevator(this);
  Climber           = new Climb(this);
  TeleopMain        = new TeleopControl(this, MecDrive, ControlElevator, Climber);
  Autos             = new Autonomous(this, MecDrive, ControlElevator, TeleopMain);
  
  Nav.Init(false);
  UltraLF.SetAutomaticMode(true);
  UltraRF.SetAutomaticMode(true);


#ifdef USB_CAMERA
  UsbCamera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  UsbCamera1.SetResolution(640, 480);
  UsbCamera1.SetFPS(24);
#endif


#ifdef AXIS_CAMERA
    AxisCamera1 = frc::CameraServer::GetInstance()->AddAxisCamera("10.25.56.17");
    AxisCamera1.SetResolution(640, 480);
    AxisCamera1.SetFPS(10);
#endif

#ifdef CAMERA
    CameraTrk.Init();
#ifdef TestValues
    frc::SmartDashboard::PutNumber("Vision Brightness", CameraTrk.ValLo);
    frc::SmartDashboard::PutNumber("Vision Display", CameraTrk.iDisplayFrame);
    pVisionThread = new std::thread(&CameraTrack::TrackThread, &CameraTrk);
#endif
#endif
#ifdef TestValues
  int period = frc::SmartDashboard::GetNumber("Shuffle Period", 1);
  //frc::SmartDashboard::PutNumber("Shuffle Period", period);// time betwwen full shuffles
  int delay = frc::SmartDashboard::GetNumber("Switch Delay", 1);
  //frc::SmartDashboard::PutNumber("Switch Delay", delay);// delay between raising and droping front pistons
  double speed = frc::SmartDashboard::GetNumber("Roller Speed", .5);
  frc::SmartDashboard::PutNumber("Roller Speed", speed);
#endif

    AutoChooser.SetDefaultOption(AutoTeleop, AutoTeleop);
    AutoChooser.AddOption(Auto1, Auto1);
    AutoChooser.AddOption(Auto2, Auto2);
    AutoChooser.AddOption(Auto3, Auto3);
    AutoChooser.AddOption(Auto4, Auto4);
    AutoChooser.AddOption(Auto5, Auto5);
    frc::SmartDashboard::PutData("Auto Selector", &AutoChooser);
  }

// ----------------------------------------------------------------------------

void Robot::RobotPeriodic() 
{
#ifdef CAMERA
#ifdef TestValues
    CameraTrk.ValLo         = frc::SmartDashboard::GetNumber("Vision Brightness", 250);
    pPrefs->PutInt("Target Val Lo", CameraTrk.ValLo);
    CameraTrk.iDisplayFrame = frc::SmartDashboard::GetNumber("Vision Display", 0);
    pPrefs->PutInt("Display Frame", CameraTrk.iDisplayFrame);
#endif
#endif
}

// ----------------------------------------------------------------------------

void Robot::AutonomousInit() 
{
  DriverCmd.CurrDriveMode = DriverCommands::DriveMode::Normal;
  Nav.SetCommandYawToCurrent(); // Put Somewhere else if we still set up at an angle Houston
  //set the elevator up
  DriverCmd.ElevatorTilted = true;

  AutoMode = AutoChooser.GetSelected();

  Autos->AutoInit();
}

// ----------------------------------------------------------------------------

void Robot::AutonomousPeriodic() 
{
  Autos->Auto(); 
  SmartDashboard::PutNumber("Motor Encoder", MotorControl_LR.GetSelectedSensorPosition());
}

// ----------------------------------------------------------------------------

void Robot::TeleopInit() 
{
  DriverCmd.CurrDriveMode = DriverCommands::DriveMode::FieldOriented;
  Nav.SetCommandYawToCurrent();
}

// ----------------------------------------------------------------------------

void Robot::TeleopPeriodic() 
{
    TeleopMain->TeleopMain();
    MecDrive->EncoderTest();
}

// ============================================================================

void Robot::TestPeriodic() 
{

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
