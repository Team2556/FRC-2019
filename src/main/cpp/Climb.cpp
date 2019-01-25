/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Climb.h"

Climb::Climb(Robot * pRobot) 
{
    this->pRobot = pRobot;
    FrontClimb = new frc::DoubleSolenoid(11,0,1);
    RearClimb  = new frc::DoubleSolenoid(11,2,3);
    
}


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

void Climb::ShuffleForward()
{
    static bool     IsMoving = false;
    static int      iCycle = 0;
    static int      iSwitch = 0;//% == 0 is front in and % == 1 is all out 
    int             ShufflePeriod = frc::SmartDashboard::GetNumber("Shuffle Period", 1);// time betwwen full shuffles
    int             SwitchDelay   = frc::SmartDashboard::GetNumber("Switch Delay", 1);// delay between raising and droping front pistons 



    frc::SmartDashboard::PutBoolean("Is Shuffling", true);
    //allow for disabling the shuffling while remaining in this function
    if (pRobot->Xbox2.GetTriggerAxis(frc::XboxController::JoystickHand::kLeftHand) > .5)
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




void Climb::Climbing()
{
    frc::SmartDashboard::PutNumber("Tilt", pRobot->pNavGyro.GetTilt());
    //frc::SmartDashboard::PutNumber("Difference", pRobot->pNavGyro.GetTilt());
    if(pRobot->Xbox2.GetAButton())
    {
        isClimbing = 1;// all out
        fInitPitch = pRobot->pNavGyro.GetTilt();
    }
    if(pRobot->Xbox2.GetBButton())
    {
        isClimbing = 0;// all in
    }
    if(pRobot->Xbox2.GetXButton())
    {
        isClimbing = 2;// front in
    }
    if(pRobot->Xbox2.GetYButton())
    {
        isClimbing = 3;// back in
    }




    if(pRobot->Xbox2.GetTriggerAxis(frc::XboxController::JoystickHand::kRightHand) > .5)
    {
     
        ShuffleForward();
    }
    else if (isClimbing == 1)// all out
    {
        frc::SmartDashboard::PutBoolean("Is Shuffling", false);
        if(fInitPitch - pRobot->pNavGyro.GetTilt()<-1.5 && ClimbCounter < 3)
        {
            FrontClimb->Set(frc::DoubleSolenoid::Value::kForward);
            Oscillation(1);


            ClimbCounter++;
        }
        else if(fInitPitch - pRobot->pNavGyro.GetTilt()>1.5 && ClimbCounter < 3)
        {
            Oscillation(0);
            RearClimb->Set(frc::DoubleSolenoid::Value::kForward);
            ClimbCounter++;
        }
        else
        {
            RearClimb->Set(frc::DoubleSolenoid::Value::kForward);
            FrontClimb->Set(frc::DoubleSolenoid::Value::kForward);
            ClimbCounter = 0;
            initBadTilt = pRobot->pNavGyro.GetTilt();
        }
        if(pRobot->Xbox2.GetBumper(frc::XboxController::JoystickHand::kLeftHand))
        {
            fInitPitch = pRobot->pNavGyro.GetTilt();
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
    else if (isClimbing == 3)// rear in
    {
        frc::SmartDashboard::PutBoolean("Is Shuffling", false);
        FrontClimb->Set(frc::DoubleSolenoid::Value::kForward);
        RearClimb->Set (frc::DoubleSolenoid::Value::kReverse);
    }
}


void Climb::test()
{

    if (pRobot->Xbox2.GetAButton())
    {
        isClimbing = 0;
    }
    else if (pRobot->Xbox2.GetBButton())
    {
        isClimbing = 1;
    }
    else if (pRobot->Xbox2.GetXButton())
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