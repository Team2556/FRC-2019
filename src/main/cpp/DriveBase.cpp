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

void DriveBase::GyroDrive()
{
    float 			fForward = 0.0;
    float 			fStrafe = 0.0;
    float			fRotate = 0.0;
    bool			bAllowRotate = false;

    // Get joystick inputs
    fForward = pRobot->DriverCmd.fMoveForward();;
    fStrafe = pRobot->DriverCmd.fMoveSideways();
    fRotate = pRobot->DriverCmd.fRotate();

    fForward = pRobot->DriverCmd.fMoveForward();;
    fStrafe = pRobot->DriverCmd.fMoveSideways();

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
        fRotate = pRobot->DriverCmd.fRotate();

        if (fRotate >  0.0) fRotate -= 0.05;
        if (fRotate <  0.0) fRotate += 0.05;
        if (fRotate >  0.5) fRotate  =  0.5;
        if (fRotate < -0.5) fRotate  = -0.5;

        pRobot->Nav.SetCommandYawToCurrent();
        frc::SmartDashboard::PutBoolean("Gryo Enabled", false);
	}
    else
    {
        // Calculate a rotation rate from robot angle error
    	fRotate = pRobot->Nav.GetRotate();
        frc::SmartDashboard::PutBoolean("Gryo Enabled", true);
    }
    frc::SmartDashboard::PutNumber("Counter", stopHoldCounter);
    frc::SmartDashboard::PutBoolean("Allow Rotate", bAllowRotate);
    frc::SmartDashboard::PutBoolean("Rotate Previous", bRotatePrevious);



    if (pRobot->DriverCmd.GetLineUp())
    {
        SmartDashboard::PutBoolean("Auto Line Up", true);
        this->RealVision(&fForward, &fStrafe, &fRotate);
    }
    else 
    {
        SmartDashboard::PutBoolean("Auto Line Up", false);
    }

    pRobot->RobotDrive.DriveCartesian(fStrafe, fForward, fRotate, 0.0);


    if(pRobot->DriverCmd.bResetGyro())
    {
        pRobot->Nav.ResetYaw();
    }
    frc::SmartDashboard::PutNumber("Distance", pRobot->UltraLF.GetRangeInches());
    


} // end GyroDrive()


// ----------------------------------------------------------------------------


void DriveBase::NormalDrive()
{
    float       fVisionTrackErrorX, fVisionTrackErrorY;
    float       fVisionTargetSizeX, fVisionTargetSizeY;
    pRobot->CameraTrk.CalcTrackError(&fVisionTrackErrorX, &fVisionTrackErrorY, &fVisionTargetSizeX, &fVisionTargetSizeY);
    pRobot->RobotDrive.DriveCartesian(
        pRobot->DriverCmd.fMoveSideways(), 
        pRobot->DriverCmd.fMoveForward(), 
        pRobot->DriverCmd.fRotate());
}

// ----------------------------------------------------------------------------

void DriveBase::FieldOrientedDrive()
{
    float 		fForward = 0.0;
    float 		fStrafe = 0.0;
    float		fRotate = 0.0;
    bool		bAllowRotate = false;

    fForward = pRobot->DriverCmd.fMoveForward();;
    fStrafe = pRobot->DriverCmd.fMoveSideways();

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
        fRotate = pRobot->DriverCmd.fRotate();

        if (fRotate >  0.0) fRotate -= 0.05;
        if (fRotate <  0.0) fRotate += 0.05;
        if (fRotate >  0.5) fRotate  =  0.5;
        if (fRotate < -0.5) fRotate  = -0.5;

        pRobot->Nav.SetCommandYawToCurrent();
        frc::SmartDashboard::PutBoolean("Gryo Enabled", false);
	}
    else
    {
        // Calculate a rotation rate from robot angle error
    	fRotate = pRobot->Nav.GetRotate();
        frc::SmartDashboard::PutBoolean("Gryo Enabled", true);
    }
    frc::SmartDashboard::PutNumber("Counter", stopHoldCounter);
    frc::SmartDashboard::PutBoolean("Allow Rotate", bAllowRotate);
    frc::SmartDashboard::PutBoolean("Rotate Previous", bRotatePrevious);

    pRobot->RobotDrive.DriveCartesian(fStrafe, fForward, fRotate, -(pRobot->Nav.GetYaw()));

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
    frc::SmartDashboard::PutBoolean("Distance Valid", bDistanceGood);


    if (pRobot->LineTracker.FrontSensors.bLineFound)
    {
        *fForward = this->LimitFWDDrive(24);
        *fStrafe = pRobot->LineTracker.GetStrafe(*fStrafe);
        
        float   fClosestAngle = FindClose(pRobot->Nav.GetYaw());
        pRobot->Nav.SetCommandYaw(fClosestAngle);
    }
    else if (bVisionTracked)
    {

        if (bVisionTracked) 
        {
            *fRotate = fVisionTrackErrorX * 0.5;
            pRobot->Nav.SetCommandYawToCurrent();
        }


        if (bDistanceGood)
        {
            float fDistanceError = fDistanceToTarget - 12.0;
            *fForward = (fDistanceError / 100.0) + 0.05;
            if (*fForward >  0.5) *fForward =  0.4;
            if (*fForward < -0.5) *fForward = -0.4;
        }
    }
}




// ----------------------------------------------------------------------------

void DriveBase::Drive()
    {
            switch (pRobot->DriverCmd.GetDriveMode())
            {
                case DriverCommands::DriveMode::Gyro :
                    frc::SmartDashboard::PutString("DriveMode", "Gyro");
                    this->GyroDrive();
                    break;

                case DriverCommands::DriveMode::FieldOriented :
                    frc::SmartDashboard::PutString("DriveMode", "Field Oriented");
                    this->FieldOrientedDrive();
                    break;

                case DriverCommands::DriveMode::DriveToTarget :
                    frc::SmartDashboard::PutString("DriveMode", "Drive To Target");
                    this->DriveToTarget();
                    break;

                case DriverCommands::DriveMode::Normal :
                default :
                    frc::SmartDashboard::PutString("DriveMode", "Normal Drive");
                    this->NormalDrive();
                    break;
            }
    } // end Drive()



// ----------------------------------------------------------------------------

float DriveBase::LimitFWDDrive(float CommandDistance)
{

    float Distance = pRobot->UltraLF.GetRangeInches();
    SmartDashboard::PutNumber("Distance", Distance);
    
    float Error = Distance - CommandDistance;

    float MaxSpeed = (Error / 100) +.05;
    SmartDashboard::PutNumber("Max Speed", MaxSpeed);
    return MaxSpeed;
}


// ----------------------------------------------------------------------------


// ============================================================================
// Local methods
// ============================================================================

float FindClose(float Angle)
{
    double Angles[8] = {61.25, 118.75, 90.0, -90.0, -118.75, -61.25, 180.0, 0.0};

    int closestIndex = 0;
    for (int i = 0; i < 8; i++)
    {
        if (fabs(Angle-Angles[i])<fabs(Angle-Angles[closestIndex]))
        {
            closestIndex = i;
        }
    }
    SmartDashboard::PutNumber("Set Angle", Angles[closestIndex]);
    return Angles[closestIndex];
}


float DriveBase::EncoderTest()
{
    float position = pRobot->MotorControl_RR.GetSelectedSensorPosition();
    SmartDashboard::PutNumber("Position", position);
    
    pRobot->MotorControl_RR.Set(ControlMode::PercentOutput, pRobot->DriverCmd.fTestValue(3));
}