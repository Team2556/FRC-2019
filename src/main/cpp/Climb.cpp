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
    FrontClimb = new frc::DoubleSolenoid(CAN_PCM,0,1); // this is the one we will be using 
    RearClimb  = new frc::DoubleSolenoid(CAN_PCM,2,3); //this one is obsolete, I just left it so it didnt break everything else
    
    
    int timing = frc::SmartDashboard::GetNumber("Timing", 3);
    frc::SmartDashboard::PutNumber("Timing", timing);
    int RearDelay = frc::SmartDashboard::GetNumber("Delay", 5);
    frc::SmartDashboard::PutNumber("Delay", RearDelay);
}


// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------

void Climb::Oscillation(int Side)
{
    int testingtime = frc::SmartDashboard::GetNumber("Timing", 3);
    if(ClimbCounter % 2 == 0)
    {
        if (Side == 0) FrontClimb->Set(frc::DoubleSolenoid::Value::kForward);
        if (Side == 1) RearClimb->Set(frc::DoubleSolenoid::Value::kForward);
    }
    else
    {
        if (Side == 0) FrontClimb->Set(frc::DoubleSolenoid::Value::kReverse);
        if (Side == 1) RearClimb->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    if(Timer % testingtime == 0)
    {
    ClimbCounter++;
    }
    Timer++;
}


// ----------------------------------------------------------------------------

void Climb::ShuffleForward()
{
    static bool     IsMoving = false;
    static int      iCycle = 0;
    static int      iSwitch = 0;//% == 0 is front in and % == 1 is all out 
    int             ShufflePeriod = frc::SmartDashboard::GetNumber("Shuffle Period", 1);// time betwwen full shuffles
    int             SwitchDelay   = frc::SmartDashboard::GetNumber("Switch Delay", 1);// delay between raising and droping front pistons 



    frc::SmartDashboard::PutBoolean("Is Shuffling", true);
    //allow for disabling the shuffling while remaining in this function
    if (pRobot->DriverCmd.fTestValue(1) > .5)
    {
        IsMoving = true;
    }
    else
    {
        IsMoving = false;
    }


    if(IsMoving)// shuffling is enabled
    {
        frc::SmartDashboard::PutBoolean("Moving", true);
        if(iSwitch % 2 == 0)// brings front in
        {
            FrontClimb->Set(frc::DoubleSolenoid::Value::kReverse);
            RearClimb->Set (frc::DoubleSolenoid::Value::kForward);
        }
        else if (iSwitch % 2 == 1)// all are out
        {
            FrontClimb->Set(frc::DoubleSolenoid::Value::kForward);
            RearClimb->Set (frc::DoubleSolenoid::Value::kForward);
        }
        

        if(iSwitch % 2 == 0 && iCycle == SwitchDelay)// if front is in and the delay to push all back out is passed
        {
            iSwitch++;
            iCycle = 0;
        }
        else if(iSwitch % 2 == 1 && iCycle == ShufflePeriod) // if front is out and the delay to pull back in is passed
        {
            iSwitch++;
            iCycle = 0;
        }
        iCycle++;
    }
    else // shuffling is disabled
    {
        frc::SmartDashboard::PutBoolean("Moving", false);
        //resets the shuffle cycle
        iCycle = 0;
        iSwitch = 0;
        // makes sure that both are pushing out
        FrontClimb->Set(frc::DoubleSolenoid::Value::kForward);
        RearClimb->Set (frc::DoubleSolenoid::Value::kForward);
    }
}


// ----------------------------------------------------------------------------

void Climb::Climbing()
{
    static int RearDelay = 0;
    static int Oscillating = 0;
    frc::SmartDashboard::PutNumber("Tilt", pRobot->Nav.GetTilt());
    frc::SmartDashboard::PutNumber("Difference", fInitPitch - pRobot->Nav.GetTilt());
    if(pRobot->DriverCmd.bTestButton(0)) // A
    {
        isClimbing = 1;// all out
        fInitPitch = pRobot->Nav.GetTilt();

    }
    if(pRobot->DriverCmd.bTestButton(1)) // B
    {
        isClimbing = 0;// all in
    }
    if(pRobot->DriverCmd.bTestButton(2)) // X
    {
        isClimbing = 2;// front in
        RearDelay = 0;
    }
    if(pRobot->DriverCmd.bTestButton(3)) // Y
    {
        isClimbing = 3;// back in
        RearDelay = 0;
    }




    
    if (isClimbing == 1)// all out
    {
        frc::SmartDashboard::PutBoolean("Is Shuffling", false);

        if (fInitPitch - pRobot->Nav.GetTilt()<-2) // robot is leaning forward and rear is higher
        {
            Oscillating = -1;
        }
        else if(fInitPitch - pRobot->Nav.GetTilt() > -1 && fInitPitch - pRobot->Nav.GetTilt() < 0)  // robot is leaning forward and rear is higher
        {
            Oscillating = 0;
        }
        else if (fInitPitch - pRobot->Nav.GetTilt() > 2) // robot is leaning backward and front is higher
        {
            Oscillating = 1;
        }
        else if (fInitPitch - pRobot->Nav.GetTilt() > 1 && fInitPitch - pRobot->Nav.GetTilt() > 0)
        {
            Oscillating = 0;
        }





        if(Oscillating == -1) // robot is leaning forward
        {
            FrontClimb->Set(frc::DoubleSolenoid::Value::kForward);
            Oscillation(1); // oscillate rear
        }
        else if(Oscillating == 1) // robot is leaning backwards
        {
            Oscillation(0);
            RearClimb->Set(frc::DoubleSolenoid::Value::kForward);
        }
        else if (Oscillating == 0)
        {
            RearClimb->Set(frc::DoubleSolenoid::Value::kForward);
            FrontClimb->Set(frc::DoubleSolenoid::Value::kForward);
            ClimbCounter = 0;
            initBadTilt = pRobot->Nav.GetTilt();
        }
    }
    else if (isClimbing == 0)// all in
    {
        frc::SmartDashboard::PutBoolean("Is Shuffling", false);
        RearClimb->Set(frc::DoubleSolenoid::Value::kReverse);
        FrontClimb->Set(frc::DoubleSolenoid::Value::kReverse);
        ClimbCounter = 0;
    }
    else if (isClimbing == 2)// front in 
    {
        frc::SmartDashboard::PutBoolean("Is Shuffling", false);
        FrontClimb->Set(frc::DoubleSolenoid::Value::kReverse);
        RearClimb->Set (frc::DoubleSolenoid::Value::kForward);
    }
    else if (isClimbing == 3)// all out without leveling
    {
        frc::SmartDashboard::PutBoolean("Is Shuffling", false);
        int Delay = frc::SmartDashboard::GetNumber("Delay", 10);
        if (RearDelay > Delay)
        {
            RearClimb->Set(frc::DoubleSolenoid::Value::kForward);
        }
        else
        {
            RearClimb->Set(frc::DoubleSolenoid::Value::kReverse);
        }
        FrontClimb->Set (frc::DoubleSolenoid::Value::kForward);
        RearDelay++;
    }
    if(pRobot->DriverCmd.bTestButton(4)) // Left bumper
    {
        fInitPitch = pRobot->Nav.GetTilt(); // reset tilt
    }
}


// ----------------------------------------------------------------------------

void Climb::test()
{

    if (pRobot->DriverCmd.bTestButton(0))
    {
        isClimbing = 0;
    }
    else if (pRobot->DriverCmd.bTestButton(1))
    {
        isClimbing = 1;
    }
    else if (pRobot->DriverCmd.bTestButton(2))
    {
        isClimbing = 2;
    }
    if(isClimbing == 0)
    {
        int testingtime = frc::SmartDashboard::GetNumber("Timing", 10);
        if(ClimbCounter % 2 == 0)
        {
            RearClimb->Set(frc::DoubleSolenoid::Value::kForward);
        }
        else
        {
            RearClimb->Set(frc::DoubleSolenoid::Value::kReverse);
        }
        if(Timer % testingtime == 0)
        {
        ClimbCounter++;
        }
        Timer++;

    }
    else if(isClimbing == 1)
    {
        RearClimb->Set(frc::DoubleSolenoid::Value::kForward);
    }
    else if(isClimbing == 2)
    {
        RearClimb->Set(frc::DoubleSolenoid::Value::kReverse);
    }

}


void Climb::HoldIn()
{
    //RearClimb->Set(frc::DoubleSolenoid::Value::kReverse);
    FrontClimb->Set(frc::DoubleSolenoid::Value::kReverse);

    if (true) // if statement so you can figure out which one works better
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




