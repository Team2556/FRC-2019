/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "frc/WPILib.h"
#include "Robot.h"
#include "DriverCommands.h"
#include "Elevator.h"

// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------

Elevator::Elevator(Robot * pRobot) 
{
    this->pRobot = pRobot;

    //create new DoubleSolenoid//
    hatchSolenoid = new frc::DoubleSolenoid(11,4,5);

    EleTilt = new frc::DoubleSolenoid(11,0,1);
}


// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------

void Elevator::ElevatorControl()
{
#if 0
    if(pRobot->Xbox2.GetTriggerAxis(frc::XboxController::kRightHand) > pRobot->Xbox2.GetTriggerAxis(frc::XboxController::kLeftHand))
    {
        ElevatorRight.Set(ControlMode::PercentOutput, pRobot->Xbox2.GetTriggerAxis(frc::XboxController::kRightHand));
        ElevatorLeft.Set(ControlMode::PercentOutput, -pRobot->Xbox2.GetTriggerAxis(frc::XboxController::kRightHand));
    }
    else 
    {
        ElevatorRight.Set(ControlMode::PercentOutput, -pRobot->Xbox2.GetTriggerAxis(frc::XboxController::kLeftHand));
        ElevatorLeft.Set(ControlMode::PercentOutput, pRobot->Xbox2.GetTriggerAxis(frc::XboxController::kLeftHand));
    }
#else
        ElevatorRight.Set(ControlMode::PercentOutput, pRobot->DriverCmd.fElevatorUpDownSpeed());
        ElevatorLeft.Set(ControlMode::PercentOutput, -pRobot->DriverCmd.fElevatorUpDownSpeed());

#endif
}


// ----------------------------------------------------------------------------
// Master function that controls everything and goes into teleop 
void Elevator::CoDriveControls()
{
    RollersControl();
    RollerIn();
    RollerOut();
    RollerLeft();
    RollerRight();
}


// ----------------------------------------------------------------------------

void Elevator::RollersControl()
{
    // Figure out whether rollers should be up or down
    if (pRobot->DriverCmd.bRollersDown())
        hatchSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    else
        hatchSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);

    // Motor control

}


// ----------------------------------------------------------------------------
//  Basic Functions to control the rollers in sync
// ----------------------------------------------------------------------------

void Elevator::RollerIn()
{
    if (pRobot->DriverCmd.bTestButton(0))
    {
        RightRoller.Set(ControlMode::PercentOutput, speed);
        LeftRoller.Set(ControlMode::PercentOutput, -speed);
    }
}


// ----------------------------------------------------------------------------

void Elevator::RollerOut()
{
    if (pRobot->DriverCmd.bTestButton(3))
    {
        RightRoller.Set(ControlMode::PercentOutput, -speed);
        LeftRoller.Set(ControlMode::PercentOutput, speed);
    }
}


// ----------------------------------------------------------------------------

void Elevator::RollerLeft()
{
    if (pRobot->DriverCmd.bTestButton(2))
    {
        RightRoller.Set(ControlMode::PercentOutput, -speed);
        LeftRoller.Set(ControlMode::PercentOutput, -speed);
    }
}


// ----------------------------------------------------------------------------

void Elevator::RollerRight()
{
    if (pRobot->DriverCmd.bTestButton(2))
    {
        RightRoller.Set(ControlMode::PercentOutput, speed);
        LeftRoller.Set(ControlMode::PercentOutput, speed);
    }
}

void Elevator::ElevatorTilt()
{
    if (pRobot->DriverCmd.bTestButton(0))
    {
        EleTilt->Set(frc::DoubleSolenoid::Value::kForward);
    }
    else if (pRobot->DriverCmd.bTestButton(1))
    {
        EleTilt->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else
    {
        EleTilt->Set(frc::DoubleSolenoid::Value::kOff);
    }
}