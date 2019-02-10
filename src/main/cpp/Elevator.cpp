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
    EleTilt = new frc::DoubleSolenoid(11,0,1);
}


// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------

void Elevator::ElevatorControl()
{
    ElevatorUpDown.Set(ControlMode::PercentOutput, pRobot->DriverCmd.fElevatorUpDownSpeed());
}


// ----------------------------------------------------------------------------
// Master function that controls everything and goes into teleop 
void Elevator::ElevatorControls()
{
    WristControl();
    RollerControl();
    ElevatorControl();
}


// ----------------------------------------------------------------------------

void Elevator::WristControl() // 
{
    // Figure out whether rollers should be up or down

    // will be added when we have a pot on the wrist
    if (pRobot->DriverCmd.bRollersDown())
    {
        //will be added when we have a pot on the wrist
        //Wrist.Set(ControlMode::Postion, DOWN);
    }
    else
    {
        //Wrist.Set(ControlMode::Postion, UP);
    }

    Wrist.Set(ControlMode::PercentOutput, pRobot->DriverCmd.fTestValue(3));// testing until we get a pot on the wrist

}


void Elevator::RollerControl()
{
    if (pRobot->DriverCmd.bTestButton(0)) // when the A button is pressed turn the rollers in
    {
        RollerIn();
    }
    else if (pRobot->DriverCmd.bTestButton(3)) // when the Y button is pressed turn the rollers out
    {
        RollerOut();
    }
    else if (pRobot->DriverCmd.bTestButton(2)) // when the X button is pressed turn the rollers left
    {
        RollerLeft();
    }
    else if (pRobot->DriverCmd.bTestButton(1)) // when the B button is pressed turn the rollers right
    {
        RollerRight();
    }
    else
    {
        RightRoller.Set(ControlMode::PercentOutput, 0);
        LeftRoller.Set(ControlMode::PercentOutput, 0);
    }
}


// ----------------------------------------------------------------------------
//  Basic Functions to control the rollers in sync
// ----------------------------------------------------------------------------

void Elevator::RollerIn()
{
    RightRoller.Set(ControlMode::PercentOutput, speed);
    LeftRoller.Set(ControlMode::PercentOutput, -speed);
}


// ----------------------------------------------------------------------------

void Elevator::RollerOut()
{
    RightRoller.Set(ControlMode::PercentOutput, -speed);
    LeftRoller.Set(ControlMode::PercentOutput, speed);
}


// ----------------------------------------------------------------------------

void Elevator::RollerLeft()
{
    RightRoller.Set(ControlMode::PercentOutput, -speed);
    LeftRoller.Set(ControlMode::PercentOutput, -speed);
}


// ----------------------------------------------------------------------------

void Elevator::RollerRight()
{
    RightRoller.Set(ControlMode::PercentOutput, speed);
    LeftRoller.Set(ControlMode::PercentOutput, speed);
}

void Elevator::ElevatorTilt(bool tiltUp)
{
    if (pRobot->DriverCmd.bElevatorTilt()) // when the driver commands the elevator to tilt, retract the piston
    {
        EleTilt->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else if (!pRobot->DriverCmd.bElevatorTilt())
    {
        EleTilt->Set(frc::DoubleSolenoid::Value::kForward);
    }
    else
    {
        EleTilt->Set(frc::DoubleSolenoid::Value::kForward);
    }
}