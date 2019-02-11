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
    this->pRobot = pRobot;
    FrontClimb = new frc::DoubleSolenoid(11,0,1);
    RearClimb  = new frc::DoubleSolenoid(11,2,6); // 6 is supposed to be 3 but the PCM on Proto is broken
    int timing = frc::SmartDashboard::GetNumber("Timing", 3);
    frc::SmartDashboard::PutNumber("Timing", timing);
    int FrontDelay = frc::SmartDashboard::GetNumber("Delay", 5);
    frc::SmartDashboard::PutNumber("Delay", FrontDelay);
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
    static int FrontDelay = 0;
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
        FrontDelay = 0;
    }
    if(pRobot->DriverCmd.bTestButton(3)) // Y
    {
        isClimbing = 3;// back in
        FrontDelay = 0;
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
        if (FrontDelay > Delay)
        {
            FrontClimb->Set(frc::DoubleSolenoid::Value::kForward);
        }
        else
        {
            FrontClimb->Set(frc::DoubleSolenoid::Value::kReverse);
        }
        RearClimb->Set (frc::DoubleSolenoid::Value::kForward);
        FrontDelay++;
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