/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "frc/WPILib.h"
#include "Robot.h"
#include "Elevator.h"

Elevator::Elevator(Robot * pRobot) 
{
    this->pRobot = pRobot;

    //create new DoubleSolenoid//
    hatchSolenoid = new frc::DoubleSolenoid(11,4,5);
}


void Elevator::ElevatorControl()
{
    if(pRobot->Xbox2.GetTriggerAxis(frc::XboxController::kRightHand)>pRobot->Xbox2.GetTriggerAxis(frc::XboxController::kLeftHand))
    {
        ElevatorRight.Set(ControlMode::PercentOutput, pRobot->Xbox2.GetTriggerAxis(frc::XboxController::kRightHand));
        ElevatorLeft.Set(ControlMode::PercentOutput, -pRobot->Xbox2.GetTriggerAxis(frc::XboxController::kRightHand));
    }
    else 
    {
        ElevatorRight.Set(ControlMode::PercentOutput, -pRobot->Xbox2.GetTriggerAxis(frc::XboxController::kLeftHand));
        ElevatorLeft.Set(ControlMode::PercentOutput, pRobot->Xbox2.GetTriggerAxis(frc::XboxController::kLeftHand));
    }
}

void Elevator::CoDriveControls()// master function that controls everything and goes into teleop 
{
    Output();
    Intake();
    RollerIn();
    RollerOut();
    RollerLeft();
    RollerRight();
}

void Elevator::Intake()
{

}


void Elevator::Output()
{
    //Pneumatic Controls
    hatchSolenoid->Set(pRobot->Xbox2.GetYButton() ? frc::DoubleSolenoid::Value::kForward : frc::DoubleSolenoid::Value::kReverse);


    //Motor Output

}




//------------------------------------------------------
//  Basic Functions to control the rollers in sync
//------------------------------------------------------


void Elevator::RollerIn()
{
    if(pRobot->Xbox2.GetAButton())
    {
        RightRoller.Set(ControlMode::PercentOutput, speed);
        LeftRoller.Set(ControlMode::PercentOutput, -speed);
    }
}
void Elevator::RollerOut()
{
    if(pRobot->Xbox2.GetYButton())
    {
        RightRoller.Set(ControlMode::PercentOutput, -speed);
        LeftRoller.Set(ControlMode::PercentOutput, speed);
    }
}
<<<<<<< HEAD



=======
void Elevator::RollerLeft()
{
    if(pRobot->Xbox2.GetXButton())
    {
        RightRoller.Set(ControlMode::PercentOutput, -speed);
        LeftRoller.Set(ControlMode::PercentOutput, -speed);
    }
}
void Elevator::RollerRight()
{
    if(pRobot->Xbox2.GetBButton())
    {
        RightRoller.Set(ControlMode::PercentOutput, speed);
        LeftRoller.Set(ControlMode::PercentOutput, speed);
    }
}
>>>>>>> f36a2818c8ab540fe32be153d75c1d8c3a6f0e55
