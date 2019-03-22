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
    EleTilt = new frc::DoubleSolenoid(11,6,5);
rollerPiston = new frc::DoubleSolenoid(11,2,3);
}


// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------

bool Elevator::ElevatorControl(DriverCommands::ElevatorHeight Height, DriverCommands::ElevatorMode Mode, int Offset = 0)
{
    if (!pRobot->DriverCmd.bTestButton(4))
    {
        ElevatorUpDown.Set(ControlMode::PercentOutput, pRobot->DriverCmd.fElevatorUpDownSpeed());
        ElevatorUpDownB.Follow(ElevatorUpDown);
        return false;
    }
    else
    {



        int iHeight = 0;
       
            if (Mode == DriverCommands::ElevatorMode::Hatch)
            {        
                switch (Height)
                {
                    case DriverCommands::ElevatorHeight::Low :
                        iHeight = LOW_HATCH;
                    break;

                    case DriverCommands::ElevatorHeight::Middle :
                        iHeight = MID_HATCH;
                    break;
                    
                    case DriverCommands::ElevatorHeight::High :
                        iHeight = TOP_HATCH;
                    break;

                    case DriverCommands::ElevatorHeight::Pickup :
                        iHeight = HATCH_PKUP;
                    break;

                    case DriverCommands::ElevatorHeight::GroundPickup :
                        iHeight = GND_HATCH_PKUP;
                    break;

                    case DriverCommands::ElevatorHeight::CargoShip :
                        iHeight = CGOSHP_HATCH;
                    break;
                }
            }
            else // elevator mode is cargo
            {
                switch (Height)
                {
                    case DriverCommands::ElevatorHeight::Low :
                        iHeight = LOW_CARGO;
                    break;

                    case DriverCommands::ElevatorHeight::Middle :
                        iHeight = MID_CARGO;
                    break;
                    
                    case DriverCommands::ElevatorHeight::High :
                        iHeight = TOP_CARGO;
                    break;

                    case DriverCommands::ElevatorHeight::Pickup :
                        iHeight = CARGO_PICKUP;
                    break;

                    case DriverCommands::ElevatorHeight::GroundPickup :
                        iHeight = GND_CARGO_PKUP;
                    break;

                    case DriverCommands::ElevatorHeight::CargoShip :
                        iHeight = CGOSHP_CARGO;
                    break;
                }
            }
        
        ElevatorUpDown.Set(ControlMode::Position, iHeight);
        ElevatorUpDownB.Follow(ElevatorUpDown);

        if(fabs(iHeight - ElevatorUpDown.GetSelectedSensorPosition()) < 8)
        {
            return true;
        }
        else 
        {
            return false;
        }
    }
}

// ----------------------------------------------------------------------------
// Master function that controls everything and goes into teleop 
void Elevator::ElevatorControls()
{
    CMDHeight = pRobot->DriverCmd.GetElevatorHeight();
    CMDMode   = pRobot->DriverCmd.GetElevatorMode();
    WristControl(CMDHeight, CMDMode);
    int ElevatorOffset = IntakeOuttake();
    ElevatorControl(CMDHeight, CMDMode, ElevatorOffset);
    ElevatorTilt(pRobot->DriverCmd.bElevatorTilt());

    frc::SmartDashboard::PutNumber("Elevator Encoder", ElevatorUpDown.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Wrist Pot", Wrist.GetSelectedSensorPosition());
}


// ----------------------------------------------------------------------------

bool Elevator::WristControl(DriverCommands::ElevatorHeight Height, DriverCommands::ElevatorMode Mode) // 
{
    
    if (!pRobot->DriverCmd.bTestButton(5))
    {
    Wrist.Set(ControlMode::PercentOutput, -(pRobot->DriverCmd.fTestValue(3)));// testing until we get a pot on the wrist
    }
    
    else
    {
        bool RollerPos; // true if up, false if down
        // Figure out whether rollers should be up or down






        if (Mode == DriverCommands::ElevatorMode::Hatch)
        {        
            switch (Height)
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

                case DriverCommands::ElevatorHeight::CargoShip :
                    RollerPos = true;
                break;
            }
        }
        else // elevator mode is cargo
        {
            switch (Height)
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

                case DriverCommands::ElevatorHeight::CargoShip :
                    RollerPos = true;
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

    }

}


int Elevator::IntakeOuttake()
{   
    int ElevatorOffset = 0;
    if(CMDMode == DriverCommands::ElevatorMode::Hatch)
    {
        //outtake
        RollerPistons(pRobot->DriverCmd.Outtake());
        if (pRobot->DriverCmd.Outtake())
        {
            //RollerIn(.3);
        }
        else
        {
            RightRoller.Set(ControlMode::PercentOutput, 0);
            LeftRoller.Set(ControlMode::PercentOutput, 0);
        }

        //intake
        if (CMDHeight == DriverCommands::ElevatorHeight::GroundPickup)
        {
            if (pRobot->HatchPickupLimitLeft.Get() && pRobot->HatchPickupLimitRight.Get())
            {
                ElevatorOffset = GND_HATCH_OFFSET;
            }
            else
            {
                ElevatorOffset = 0;
            }
            
        }

    }
    else // cargo mode
    {
        speed = frc::SmartDashboard::GetNumber("Roller Speed", .5);
        if (pRobot->DriverCmd.Intake()) // when the A button is pressed turn the rollers in
        {
            if (CMDHeight == DriverCommands::ElevatorHeight::GroundPickup)
            {
                ElevatorOffset = GND_CARGO_OFFSET;
            }
            RollerIn(speed);
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
        rollerPiston->Set(frc::DoubleSolenoid::Value::kForward);
    }

    if (CMDMode == DriverCommands::ElevatorMode::Hatch && CMDHeight == DriverCommands::ElevatorHeight::GroundPickup)
    {
        if (pRobot->HatchPickupLimitLeft.Get() && pRobot->HatchPickupLimitRight.Get())
        {
            frc::SmartDashboard::PutString("Hatch Lineup Guide", "Good");
        }
        else if (pRobot->HatchPickupLimitLeft.Get())
        {
            frc::SmartDashboard::PutString("Hatch Lineup Guide", "Turn Right");
        }
        else if (pRobot->HatchPickupLimitRight.Get())
        {
            frc::SmartDashboard::PutString("Hatch Lineup Guide", "Turn Left");
        }
        else
        {
            frc::SmartDashboard::PutString("Hatch Lineup Guide", "Go Forward");
        }
    }
    else
    {
        frc::SmartDashboard::PutString("Hatch Lineup Guide", "Not Hatch Pickup");
    }

    return ElevatorOffset;
}


// ----------------------------------------------------------------------------
//  Basic Functions to control the rollers in sync
// ----------------------------------------------------------------------------

void Elevator::RollerIn(float Speed)
{
    RightRoller.Set(ControlMode::PercentOutput, Speed);
    LeftRoller.Set(ControlMode::PercentOutput, -Speed);
}


// ----------------------------------------------------------------------------

void Elevator::RollerOut()
{
    RightRoller.Set(ControlMode::PercentOutput, -1);
    LeftRoller.Set(ControlMode::PercentOutput, 1);
}


// ----------------------------------------------------------------------------

void Elevator::ElevatorTilt(bool Position)
{
    if (Position) // when the driver commands the elevator to tilt, retract the piston
    {
        EleTilt->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else if (!Position)
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
}



float Elevator::EncoderTest()
{
    ElevatorUpDown.Set(ControlMode::PercentOutput, pRobot->DriverCmd.fElevatorUpDownSpeed());
    ElevatorUpDownB.Follow(ElevatorUpDown);
}