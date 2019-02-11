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
    EleTilt = new frc::DoubleSolenoid(11,4,5);
    rollerPiston = new frc::DoubleSolenoid(11,6,7);
}


// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------

void Elevator::ElevatorControl()
{
    #define     MANUAL
    
    #ifdef MANUAL

    ElevatorUpDown.Set(ControlMode::PercentOutput, pRobot->DriverCmd.fElevatorUpDownSpeed());
    #else
    
    if (pRobot->DriverCmd.fElevatorUpDownSpeed() > .05)
    {
        ElevatorUpDown.Set(ControlMode::PercentOutput, pRobot->DriverCmd.fElevatorUpDownSpeed());
    }
    else
    {
        if (CMDMode == DriverCommands::ElevatorMode::Hatch)
        {        
            switch (CMDHeight)
            {
                case DriverCommands::ElevatorHeight::Low :
                    ElevatorUpDown.Set(ControlMode::Position, LOW_HATCH);
                break;

                case DriverCommands::ElevatorHeight::Middle :
                    ElevatorUpDown.Set(ControlMode::Position, MID_HATCH);
                break;
                
                case DriverCommands::ElevatorHeight::High :
                    ElevatorUpDown.Set(ControlMode::Position, TOP_HATCH);
                break;

                case DriverCommands::ElevatorHeight::Pickup :
                    ElevatorUpDown.Set(ControlMode::Position, HATCH_PKUP);
                break;

                case DriverCommands::ElevatorHeight::GroundPickup :
                    ElevatorUpDown.Set(ControlMode::Position, GND_HATCH_PKUP);
                break;
            }
        }
        else // elevator mode is cargo
        {
            switch (CMDHeight)
            {
                case DriverCommands::ElevatorHeight::Low :
                    ElevatorUpDown.Set(ControlMode::Position, LOW_CARGO);
                break;

                case DriverCommands::ElevatorHeight::Middle :
                    ElevatorUpDown.Set(ControlMode::Position, MID_CARGO);
                break;
                
                case DriverCommands::ElevatorHeight::High :
                    ElevatorUpDown.Set(ControlMode::Position, TOP_CARGO);
                break;

                case DriverCommands::ElevatorHeight::Pickup :
                    ElevatorUpDown.Set(ControlMode::Position, CARGO_PICKUP);
                break;

                case DriverCommands::ElevatorHeight::GroundPickup :
                    ElevatorUpDown.Set(ControlMode::Position, GND_CARGO_PKUP);
                break;
            }
        }
    }

    #endif

}

// ----------------------------------------------------------------------------
// Master function that controls everything and goes into teleop 
void Elevator::ElevatorControls()
{
    CMDHeight = pRobot->DriverCmd.GetElevatorHeight();
    CMDMode   = pRobot->DriverCmd.GetElevatorMode();
    WristControl();
    RollerControl();
    ElevatorControl();
    ElevatorTilt();
}


// ----------------------------------------------------------------------------

void Elevator::WristControl() // 
{
    // Figure out whether rollers should be up or down

    // will be added when we have a pot on the wrist
    if (pRobot->DriverCmd.bRollersDown())
    {
        //will be added when we have a pot on the wrist
        //Wrist.Set(ControlMode::Postion, WRIST_DOWN);
    }
    else
    {
        //Wrist.Set(ControlMode::Postion, WRIST_UP);
    }

    Wrist.Set(ControlMode::PercentOutput, pRobot->DriverCmd.fTestValue(3));// testing until we get a pot on the wrist

}


void Elevator::RollerControl()
{   
    RollerPistons();

    speed = frc::SmartDashboard::GetNumber("Roller Speed", 1);
    if (pRobot->DriverCmd.bTestButton(0)) // when the A button is pressed turn the rollers in
    {
        RollerIn();
    }
    else if (pRobot->DriverCmd.bTestButton(3)) // when the Y button is pressed turn the rollers out
    {
        RollerOut();
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

void Elevator::ElevatorTilt()
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

void Elevator::RollerPistons()
{
    if (pRobot->DriverCmd.bRollerPistons()) // when the driver commands the elevator to tilt, retract the piston
    {
        rollerPiston->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else if (!pRobot->DriverCmd.bRollerPistons())
    {
        rollerPiston->Set(frc::DoubleSolenoid::Value::kForward);
    }
    else
    {
        rollerPiston->Set(frc::DoubleSolenoid::Value::kReverse);
    }
}