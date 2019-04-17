/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Climb.h"

// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------

Climb::Climb(Robot * pRobot) 
{
    //For the purposes of this code, the front is the side that doesnt have the elevator and the back is the elevator side

    this->pRobot = pRobot;
    FrontClimb = new frc::DoubleSolenoid(CAN_PCM,2,3); // this is the one we will be using 
    //RearClimb  = new frc::DoubleSolenoid(CAN_PCM,2,3); //this one is obsolete, I just left it so it didnt break everything else
    
    
}


// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------


void Climb::HoldIn()
{
    //RearClimb->Set(frc::DoubleSolenoid::Value::kReverse);
    FrontClimb->Set(frc::DoubleSolenoid::Value::kReverse);

    if (false) // if statement so you can figure out which one works better
    {
        ClimbMotor.Set(ControlMode::Position, 0.0); // this one requres an encoder on talon 10
        ClimbMotorB.Follow(ClimbMotor); // encoder will always be set to 0 at power on so this will hold it where it started
    }
    else
    {
        ClimbMotor.Set(ControlMode::PercentOutput, 0.0); // if you use this make sure both talons are set to break mode 
        ClimbMotorB.Follow(ClimbMotor);
    }
}


void Climb::NewClimbing()
{
    double ClimbMotorSpeed = 0.0;
    #define BASE_CLIMB_SPEED    .5 // this should be the value that you have to pass into the motor to have it roughly keep up with the piston
    #define MAX_CLIMB_ENCODER_POS   1000

    if (pRobot->DriverCmd.bCurrentlyClimbing()) // we are going up
    {
        FrontClimb->Set(frc::DoubleSolenoid::Value::kForward);

        ClimbMotorSpeed = BASE_CLIMB_SPEED + .05 * pRobot->Nav.GetTiltError();
        /* 
        The logic for this is go a constant speed and speed up or slow down based on
        how bad the tilt is
        
        The . 5 is a test value that is probably way to high
        */
        // speed may need to be negative depending on the way the motors are set up
        
        if (ClimbMotor.GetSelectedSensorPosition() < MAX_CLIMB_ENCODER_POS)
        {
            ClimbMotor.Set(ControlMode::PercentOutput, ClimbMotorSpeed); // just run the motor at the previously calculated speed
            ClimbMotorB.Follow(ClimbMotor);
        }
        else
        {
            ClimbMotor.Set(ControlMode::PercentOutput, 0.0); // once agian make sure both talons are on break mode
            ClimbMotorB.Follow(ClimbMotor);
        } 
    }
    else // not climbing right now
    {
        this->HoldIn();
    }

    if (pRobot->DriverCmd.bTestButton(3)) // reset tilt on Y button press
    {
        pRobot->Nav.ResetTilt();
    }
}


void Climb::Climb2()
{
    if (pRobot->DriverCmd.bTestButton(1))
    {
        FrontClimb->Set(frc::DoubleSolenoid::Value::kForward);
    }
    else
    {
        FrontClimb->Set(frc::DoubleSolenoid::Value::kReverse);
    }


    ClimbMotor.Set(ControlMode::PercentOutput, pRobot->DriverCmd.fClimbScrewSpeed());
}


