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

void Elevator::ElevatorControl(int Offset = 0)
{
    //
    #define     MANUAL
    
    #ifdef MANUAL

    ElevatorUpDown.Set(ControlMode::PercentOutput, pRobot->DriverCmd.fElevatorUpDownSpeed());
    #else
    
    int Height = 0;

    
    
        if (CMDMode == DriverCommands::ElevatorMode::Hatch)
        {        
            switch (CMDHeight)
            {
                case DriverCommands::ElevatorHeight::Low :
                    Height = LOW_HATCH;
                break;

                case DriverCommands::ElevatorHeight::Middle :
                    Height = MID_HATCH;
                break;
                
                case DriverCommands::ElevatorHeight::High :
                    Height = TOP_HATCH;
                break;

                case DriverCommands::ElevatorHeight::Pickup :
                    Height = HATCH_PKUP;
                break;

                case DriverCommands::ElevatorHeight::GroundPickup :
                    Height = GND_HATCH_PKUP;
                break;
            }
        }
        else // elevator mode is cargo
        {
            switch (CMDHeight)
            {
                case DriverCommands::ElevatorHeight::Low :
                    Height = LOW_CARGO;
                break;

                case DriverCommands::ElevatorHeight::Middle :
                    Height = MID_CARGO;
                break;
                
                case DriverCommands::ElevatorHeight::High :
                    Height = TOP_CARGO;
                break;

                case DriverCommands::ElevatorHeight::Pickup :
                    Height = CARGO_PICKUP;
                break;

                case DriverCommands::ElevatorHeight::GroundPickup :
                    Height = GND_CARGO_PKUP;
                break;
            }
        }
    

    ElevatorUpDown.Set(ControlMode::Position, Height);
    #endif

}

// ----------------------------------------------------------------------------
// Master function that controls everything and goes into teleop 
void Elevator::ElevatorControls()
{
    CMDHeight = pRobot->DriverCmd.GetElevatorHeight();
    CMDMode   = pRobot->DriverCmd.GetElevatorMode();
    WristControl();
    //int ElevatorOffset = IntakeOuttake();
    //ElevatorControl(ElevatorOffset);
    ElevatorTilt();

    frc::SmartDashboard::PutNumber("Elevator Encoder", ElevatorUpDown.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Wrist Pot", Wrist.GetSelectedSensorPosition());
}


// ----------------------------------------------------------------------------

void Elevator::WristControl() // 
{
    #ifdef MANUAL

    Wrist.Set(ControlMode::PercentOutput, -(pRobot->DriverCmd.fTestValue(3)));// testing until we get a pot on the wrist

    #else
    bool RollerPos; // true if up, false if down
    // Figure out whether rollers should be up or down






    if (CMDMode == DriverCommands::ElevatorMode::Hatch)
    {        
        switch (CMDHeight)
        {
            case DriverCommands::ElevatorHeight::Low :
                RollerPos = true;
            break;

            case DriverCommands::ElevatorHeight::Middle :
                RollerPos = true;
            break;
            
            case DriverCommands::ElevatorHeight::High :
                RollerPos = true;
            break;

            case DriverCommands::ElevatorHeight::Pickup :
                RollerPos = true;
            break;

            case DriverCommands::ElevatorHeight::GroundPickup :
                RollerPos = false;
            break;
        }
    }
    else // elevator mode is cargo
    {
        switch (CMDHeight)
        {
            case DriverCommands::ElevatorHeight::Low :
                RollerPos = true;
            break;

            case DriverCommands::ElevatorHeight::Middle :
                RollerPos = true;
            break;
            
            case DriverCommands::ElevatorHeight::High :
                RollerPos = true;
            break;

            case DriverCommands::ElevatorHeight::Pickup :
                RollerPos = true;
            break;

            case DriverCommands::ElevatorHeight::GroundPickup :
                RollerPos = false;
            break;
        }
    }



    
    if (RollerPos)
    {
        Wrist.Set(ControlMode::Position, WRIST_UP);
    }
    else if (!RollerPos)
    {
        Wrist.Set(ControlMode::Position, WRIST_DOWN);
    }

    #endif

}


int Elevator::IntakeOuttake()
{   
    int ElevatorOffset = 0;
    if(CMDMode == DriverCommands::ElevatorMode::Hatch)
    {
        //outtake
        RollerPistons(pRobot->DriverCmd.Outtake());

        //intake
        if (pRobot->DriverCmd.Intake())
        {
            if (CMDHeight == DriverCommands::ElevatorHeight::GroundPickup)
            {
                ElevatorOffset = GND_HATCH_OFFSET;
            }
        }

        //turn off rollers
        RightRoller.Set(ControlMode::PercentOutput, 0);
        LeftRoller.Set(ControlMode::PercentOutput, 0);
    }
    else // cargo mode
    {
        
        speed = frc::SmartDashboard::GetNumber("Roller Speed", 1);
        if (pRobot->DriverCmd.Intake()) // when the A button is pressed turn the rollers in
        {
            if (CMDHeight == DriverCommands::ElevatorHeight::GroundPickup)
            {
                ElevatorOffset = GND_CARGO_OFFSET;
            }
            RollerIn();
        }
        else if (pRobot->DriverCmd.Outtake()) // when the Y button is pressed turn the rollers out
        {
            RollerOut();
        }
        else
        {
            RightRoller.Set(ControlMode::PercentOutput, 0);
            LeftRoller.Set(ControlMode::PercentOutput, 0);
        }

        // make sure the hatch pistons are in
        rollerPiston->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    return ElevatorOffset;
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
    RightRoller.Set(ControlMode::PercentOutput, -1);
    LeftRoller.Set(ControlMode::PercentOutput, 1);
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

void Elevator::RollerPistons(bool bHatchOut)
{
    if (bHatchOut) // when the driver commands the elevator to tilt, retract the piston
    {
        rollerPiston->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else if (!bHatchOut)
    {
        rollerPiston->Set(frc::DoubleSolenoid::Value::kForward);
    }
    else
    {
        rollerPiston->Set(frc::DoubleSolenoid::Value::kReverse);
    }
}



float Elevator::EncoderTest()
{
    float position = ElevatorUpDown.GetSelectedSensorPosition();
    SmartDashboard::PutNumber("Position", position);
    
    ElevatorUpDown.Set(ControlMode::PercentOutput, pRobot->DriverCmd.fElevatorUpDownSpeed());
}