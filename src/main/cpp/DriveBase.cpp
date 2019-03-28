/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "DriverCommands.h"
#include "DriveBase.h"

float FindClose(float Angle);

// ============================================================================
// Class DriveBase
// ============================================================================

//Constructor
DriveBase::DriveBase(Robot * pRobot) 
{
    this->pRobot = pRobot;
}


// All comments are in field oriented drive

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------

void DriveBase::GyroDrive(float * fForward, float * fStrafe, float *fRotate)
{
    bool			bAllowRotate = false;

    // Get joystick inputs
    *fForward = pRobot->DriverCmd.fMoveForward();
    *fStrafe = pRobot->DriverCmd.fMoveSideways();
    *fRotate = pRobot->DriverCmd.fRotate();


    if(pRobot->DriverCmd.bManualRotate())
    {
        bAllowRotate = true;
    }

    if((pRobot->DriverCmd.POV() > -1) && (pRobot->Nav.bPresetTurning == false))
    {
    	pRobot->Nav.fGyroCommandYaw = pRobot->DriverCmd.POV();
    	pRobot->Nav.bPresetTurning = true;
        bAllowRotate = false;
    }
    if(fabs(pRobot->Nav.GetYawError())<10)
    {
    	pRobot->Nav.bPresetTurning = false;
    }
    else if(pRobot->Nav.bPresetTurning == true)
    {
        bAllowRotate = false;
    }

    if(bAllowRotate == true)
    {
        bRotatePrevious = true;
    }

    if((bAllowRotate == false) && (bRotatePrevious == true) && (stopHoldCounter < 5))
    {
        stopHoldCounter++;
    }
    else if((bAllowRotate == false) && (bRotatePrevious == true) && (stopHoldCounter >= 5))
    {
        stopHoldCounter = 0;
        bRotatePrevious = false;
    }


    if (bRotatePrevious)
	{
        *fRotate = pRobot->DriverCmd.fRotate();

        if (*fRotate >  0.0) *fRotate -= 0.05;
        if (*fRotate <  0.0) *fRotate += 0.05;
        if (*fRotate >  0.5) *fRotate  =  0.5;
        if (*fRotate < -0.5) *fRotate  = -0.5;

        pRobot->Nav.SetCommandYawToCurrent();
        frc::SmartDashboard::PutBoolean("Gryo Enabled", false);
	}
    else
    {
        // Calculate a rotation rate from robot angle error
    	*fRotate = pRobot->Nav.GetRotate();
        frc::SmartDashboard::PutBoolean("Gryo Enabled", true);
    }



    if(pRobot->DriverCmd.bResetGyro())
    {
        pRobot->Nav.ResetYaw();
    }
    


} // end GyroDrive()


// ----------------------------------------------------------------------------


void DriveBase::NormalDrive(float * fForward, float * fStrafe, float *fRotate)
{
    
    pRobot->RobotDrive.DriveCartesian(
        pRobot->DriverCmd.fMoveSideways(), 
        pRobot->DriverCmd.fMoveForward(), 
        pRobot->DriverCmd.fRotate());
}

// ----------------------------------------------------------------------------

void DriveBase::FieldOrientedDrive(float * fForward, float * fStrafe, float *fRotate, bool * bFOD)
{
    *bFOD = true;
    bool		bAllowRotate = false;

    *fForward = pRobot->DriverCmd.fMoveForward();;
    *fStrafe = pRobot->DriverCmd.fMoveSideways();

    if(pRobot->DriverCmd.bManualRotate())
    {
        bAllowRotate = true;
    }

    if((pRobot->DriverCmd.POV() > -1) && (pRobot->Nav.bPresetTurning == false))
    {
    	pRobot->Nav.fGyroCommandYaw = pRobot->DriverCmd.POV();
    	pRobot->Nav.bPresetTurning = true;
        bAllowRotate = false;
    }
    if(fabs(pRobot->Nav.GetYawError())<10)
    {
    	pRobot->Nav.bPresetTurning = false;
    }
    else if(pRobot->Nav.bPresetTurning == true)
    {
        bAllowRotate = false;
    }

    if(bAllowRotate == true)
    {
        bRotatePrevious = true;
    }

    if((bAllowRotate == false) && (bRotatePrevious == true) && (stopHoldCounter < 5))
    {
        stopHoldCounter++;
    }
    else if((bAllowRotate == false) && (bRotatePrevious == true) && (stopHoldCounter >= 5))
    {
        stopHoldCounter = 0;
        bRotatePrevious = false;
    }


    if (bRotatePrevious)
	{
        *fRotate = pRobot->DriverCmd.fRotate();

        if (*fRotate >  0.0) *fRotate -= 0.05;
        if (*fRotate <  0.0) *fRotate += 0.05;
        if (*fRotate >  0.5) *fRotate  =  0.5;
        if (*fRotate < -0.5) *fRotate  = -0.5;

        pRobot->Nav.SetCommandYawToCurrent();
        frc::SmartDashboard::PutBoolean("Gryo Enabled", false);
	}
    else
    {
        // Calculate a rotation rate from robot angle error
    	*fRotate = pRobot->Nav.GetRotate();
        frc::SmartDashboard::PutBoolean("Gryo Enabled", true);
    }


    frc::SmartDashboard::PutBoolean("Allow Rotate", bAllowRotate);


    //pRobot->RobotDrive.DriveCartesian(fStrafe, fForward, fRotate, -(pRobot->Nav.GetYaw()));

    if(pRobot->DriverCmd.bResetGyro())
    {
        pRobot->Nav.ResetYaw();
    }
} // end FieldOrientedDrive()


// ----------------------------------------------------------------------------

void DriveBase::DriveToTarget()
    {
    static int  iDriveToTargetState = 0;
    float 		fStrafe;
    float 		fForward;
    float		fRotate;
    bool		bAllowRotate = false;
    bool        bVisionTracked;
    float       fVisionTrackErrorX, fVisionTrackErrorY;
    float       fVisionTargetSizeX, fVisionTargetSizeY;
    char        szVisionMsg[100];

    bool        bDistanceGood;
    float       fDistanceToTarget;
    char        szPrint[100];

    /* Drive States
         0  Get ready to drive
        10  Rotate based on vision target
        20  Drive towards target until near
        30  Final drive to deploy
      1000  Done, manual control
    */

    // Get drive inputs
    fForward = pRobot->DriverCmd.fMoveForward();
    fStrafe = pRobot->DriverCmd.fMoveSideways();

    // Get vision track status and track errors
    pRobot->CameraTrk.GetTrackError(&bVisionTracked, &fVisionTrackErrorX, &fVisionTrackErrorY, &fVisionTargetSizeX, &fVisionTargetSizeY);

    // Get ultrasonic distance
#if 1
    fDistanceToTarget = 0.0;
    fDistanceToTarget += pRobot->UltraLF.GetRangeInches();
    fDistanceToTarget += pRobot->UltraLF.GetRangeInches();
    fDistanceToTarget += pRobot->UltraLF.GetRangeInches();
    fDistanceToTarget =  fDistanceToTarget / 3.0;
#else
    fDistanceToTarget = Ultra.GetRangeInches();
#endif
    bDistanceGood     =  pRobot->UltraLF.IsRangeValid();
    frc::SmartDashboard::PutBoolean("Distance Valid", bDistanceGood);
    if (bDistanceGood) frc::SmartDashboard::PutNumber("Distance", fDistanceToTarget);
    else               frc::SmartDashboard::PutString("Distance", " ");

    // Get line track error
    

    // Get default values for rotate
    if(pRobot->DriverCmd.bManualRotate())
        {
        fRotate = pRobot->DriverCmd.fRotate();
        pRobot->Nav.SetCommandYawToCurrent();
//            sprintf(szVisionMsg, "MANUAL  %4.1f", fRotate);
        }
    // Gyro control rotate
    else
        {
        fRotate = pRobot->Nav.GetRotate();
//            sprintf(szVisionMsg, "GYRO    %4.1f", fRotate);
        }

    // If auto-drive button not pressed then just manual control
    // ---------------------------------------------------------
    if (pRobot->DriverCmd.bTestButton(0) == false)
        {
        iDriveToTargetState = 0;
        } // end if manual drive

    // Auto-drive button pressed so, um.... auto-drive
    // -----------------------------------------------
    else 
        {
        switch (iDriveToTargetState)
            {
            case 0 :
                iDriveToTargetState = 10;
                // No break!

            case 10 :
                // Rotate
                if (bVisionTracked) 
                {
                fRotate = fVisionTrackErrorX * 0.5;
                pRobot->Nav.SetCommandYawToCurrent();
                }
                // else default rotate value
                
                // Forward
                if (bDistanceGood)
                {
                float fDistanceError = fDistanceToTarget - 12.0;
                fForward = (fDistanceError / 100.0) + 0.05;
                if (fForward >  0.5) fForward =  0.4;
                if (fForward < -0.5) fForward = -0.4;
                }
                // else fXStick stays from controller

                // Strafe

                // Exit to next state
                if ((bDistanceGood == true) && (fDistanceToTarget < 12.0))
                    iDriveToTargetState = 1000;

                break;

            case 1000 : // Done, manual control
                break;

            default :   // Unknown state, manual control
                break;
            } // end switch on drive state
        } // end if auto-drive

//            fRotate = pRobot->Nav.GetRotate();
//            sprintf(szVisionMsg, "NO TRACK %4.1f", fRotate);

    // Drive the robot
    sprintf(szPrint, "%4d X %5.1f Y %5.1f R %5.1f\n", iDriveToTargetState, fStrafe, fForward, fRotate);
    frc::SmartDashboard::PutString("Drive Controls", szPrint);
    pRobot->RobotDrive.DriveCartesian(fStrafe, fForward, fRotate, 0.0);

    } // end DriveToTarget()


// ----------------------------------------------------------------------------

void DriveBase::RealVision(float * fForward, float * fStrafe, float *fRotate)
{
    static int StopCounter = 0;
    bool        bVisionTracked;
    float       fVisionTrackErrorX, fVisionTrackErrorY;
    float       fVisionTargetSizeX, fVisionTargetSizeY;


    bool        bDistanceGood;
    float       fDistanceToTarget;

    pRobot->CameraTrk.GetTrackError(&bVisionTracked, &fVisionTrackErrorX, &fVisionTrackErrorY, &fVisionTargetSizeX, &fVisionTargetSizeY);


    fDistanceToTarget = 0.0;
    fDistanceToTarget += pRobot->UltraLF.GetRangeInches();
    fDistanceToTarget += pRobot->UltraLF.GetRangeInches();
    fDistanceToTarget += pRobot->UltraLF.GetRangeInches();
    fDistanceToTarget =  fDistanceToTarget / 3.0;

    bDistanceGood     =  pRobot->UltraLF.IsRangeValid();
    
    if (!bVisionTracked && !pRobot->LineTracker.FrontSensors.bLineFound)
    {
        State = 0;
    }


    switch (State)
    {
        case 0 : // pre state

            if (bVisionTracked)
            {
                State = 10;
            }
            else if (pRobot->LineTracker.FrontSensors.bLineFound)
            {
                State = 20;
            }

        break;

        case 10 :
            if(!bVisionTracked || bVisionTracked)
            {
                *fRotate = fVisionTrackErrorX * 0.35;
                pRobot->Nav.SetCommandYawToCurrent();
            
            if (fDistanceToTarget > 18.5)
            {
                *fForward = this ->LimitFWDDrive(18);
            
            }
            if(pRobot->LineTracker.FrontSensors.bLineFound)
            {
                State = 15;
            }

            if(fDistanceToTarget < 18.25)
            {
                State = 0;
            }
            }
        break;

        case 15 :

            *fForward = -.1;
            *fStrafe = .05;

            StopCounter++;

            if (StopCounter >= 3)
            {
                State = 17;
                StopCounter = 0;
            }
        break;

        case 17 :
            *fForward = 0;
            *fStrafe = 0;
            pRobot->Nav.SetCommandYaw(FindClose(pRobot->Nav.GetYaw()));

            if (fabs(pRobot->Nav.GetYawError()) < 4)
            {
                State = 20;
            }
        break;

        case 20 : 
            *fForward = this->LimitFWDDrive(18);
            *fStrafe = pRobot->LineTracker.GetStrafe(*fStrafe);

            
            

            if (pRobot->LineTracker.GetStrafe(*fStrafe) == 0)
            {
                StopCounter++;
            }
            else 
            {
                StopCounter = 0;
            }

            if (StopCounter >= 5)
            {
                State = 30;
                StopCounter = 0;
            }
        break;

        case 30 :
            *fForward = this->LimitFWDDrive(9);
            *fStrafe = 0;
        break;


    }
    SmartDashboard::PutNumber("State", State);
}




// ----------------------------------------------------------------------------

void DriveBase::Drive(float fForward, float fStrafe, float fRotate, bool bFOD)
{
    if (bFOD)// field oriented drive is enabled
    {
        pRobot->RobotDrive.DriveCartesian(fStrafe, fForward, fRotate, -(pRobot->Nav.GetYaw()));
    }
    else // field oriented drive is disabled
    {   
        pRobot->RobotDrive.DriveCartesian(fStrafe, fForward, fRotate, 0.0);
    }
} // end Drive()



// ----------------------------------------------------------------------------

float DriveBase::LimitFWDDrive(float CommandDistance)
{

    float Distance = pRobot->UltraLF.GetRangeInches();
    SmartDashboard::PutNumber("Distance", Distance);
    
    float Error = Distance - CommandDistance;

    float MaxSpeed = (Error / 90) +(.03* (Error/fabs(Error)));

    if (MaxSpeed > .6) MaxSpeed = .6;
    if (MaxSpeed < -.6) MaxSpeed = -.6;
    SmartDashboard::PutNumber("Max Speed", MaxSpeed);
    return MaxSpeed;
}


// ----------------------------------------------------------------------------


// ============================================================================
// Local methods
// ============================================================================

float DriveBase::FindClose(float Angle)
{
    double RocketHatchAngles[] = {-28.75, -151.25, 28.75, 151.25};
    double RocketCargoAngles[] = {90, -90};
    double CargoshipAngles[] = {90, -90, 0};
    double HumanplayerAngles[]= {180};

    double Aglet; //= Angle;

    int closestIndex = 0;


    if (pRobot->DriverCmd.GetElevatorHeight() == DriverCommands::ElevatorHeight::CargoShip)
    {
        for (int i = 0; i < 3; i++)
        {
            if(fabs(Angle-CargoshipAngles[i])<fabs(Angle-CargoshipAngles[closestIndex]))
            {
                closestIndex = i;
            }
        }
        Aglet = CargoshipAngles[closestIndex];
        frc::SmartDashboard::PutString("Array", "Cargo Ship");
    }
    else if (pRobot->DriverCmd.GetElevatorHeight() == DriverCommands::ElevatorHeight::Pickup)
    {
        for (int i = 0; i < 1; i++)
        {   
            if(fabs(Angle-HumanplayerAngles[i])<fabs(Angle-HumanplayerAngles[closestIndex]))
            {
                closestIndex = i;
            }
        }
        Aglet = HumanplayerAngles[closestIndex];
        frc::SmartDashboard::PutString("Array", "Pickup");
    }
    else if(pRobot->DriverCmd.GetElevatorMode() == DriverCommands::ElevatorMode::Cargo &&  pRobot->DriverCmd.GetElevatorHeight() != DriverCommands::ElevatorHeight::GroundPickup)
    {
        for (int i = 0; i < 2; i++)
        {
            if(fabs(Angle-RocketCargoAngles[i])<fabs(Angle-RocketCargoAngles[closestIndex]))
            {
                closestIndex = i;
            }
        }
        Aglet = RocketCargoAngles[closestIndex];
        frc::SmartDashboard::PutString("Array", "Cargo");
    }
    else if(pRobot->DriverCmd.GetElevatorMode() == DriverCommands::ElevatorMode::Hatch && pRobot->DriverCmd.GetElevatorHeight() != DriverCommands::ElevatorHeight::GroundPickup)
    {
        for (int i = 0; i < 4; i++)
        {
            if(fabs(Angle-RocketHatchAngles[i])<fabs(Angle-RocketHatchAngles[closestIndex]))
            {
                closestIndex = i;
            }
        }
        Aglet = RocketHatchAngles[closestIndex];
        frc::SmartDashboard::PutString("Array", "Hatch");
    }
            
    return Aglet;
    
    SmartDashboard::PutNumber("Set Angle", Aglet);
}


float DriveBase::EncoderTest()
{
    float position = pRobot->MotorControl_RR.GetSelectedSensorPosition();
    SmartDashboard::PutNumber("Position", position);
    
    pRobot->MotorControl_RR.Set(ControlMode::PercentOutput, pRobot->DriverCmd.fTestValue(3));
}