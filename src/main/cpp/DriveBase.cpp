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

    if ((pRobot->DriverCmd.POV() > -1) && (pRobot->Nav.bPresetTurning == false))
    {
    	pRobot->Nav.fGyroCommandYaw = pRobot->Nav.fGyroCommandYaw + pRobot->DriverCmd.POV();
    	pRobot->Nav.bPresetTurning = true;
    }

    // If small pointing error then 
    if (fabs(pRobot->Nav.GetYawError()) < 10.0)
    {
    	pRobot->Nav.bPresetTurning = false;
    }

    // If the right trigger is pressed then rotate is controlled by right stick
    bAllowRotate = pRobot->DriverCmd.bManualRotate();
    
    if (bAllowRotate)
	{
        // Get rotation rate from the right stick
        fRotate = pRobot->DriverCmd.fRotate();
#if 0
        fRotate = pRobot->Nav.CorrectRotate(fRotate);
#else
        if (fRotate >  0.5) fRotate =  0.5;
        if (fRotate < -0.5) fRotate = -0.5;
#endif
        // Set commanded angle to whatever the current angle is
        pRobot->Nav.SetCommandYawToCurrent();
	}
    else
    {
        // Calculate a rotation rate from robot angle error
    	fRotate = pRobot->Nav.GetRotate();
    }

    fForward = this->LimitFWDDrive(fForward, false, 12);
    fStrafe = pRobot->LineTracker.GetStrafe(fStrafe, pRobot->DriverCmd.GetLineUpStrafe());
    if (pRobot->LineTracker.BackSensors.bLineFound && pRobot->DriverCmd.GetLineUpStrafe())
    {
        float   fClosestAngle = FindClose(pRobot->Nav.GetYaw());
        pRobot->Nav.SetCommandYaw(fClosestAngle);
    }
    pRobot->RobotDrive.DriveCartesian(fStrafe, fForward, fRotate, 0.0);


    if(pRobot->DriverCmd.bResetGyro())
    {
        pRobot->Nav.ResetYaw();
    }
} // end GyroDrive()


// ----------------------------------------------------------------------------


void DriveBase::NormalDrive()
{
    pRobot->RobotDrive.DriveCartesian(
        pRobot->DriverCmd.fMoveForward(), 
        pRobot->DriverCmd.fMoveSideways(), 
        pRobot->DriverCmd.fRotate());
}


// ----------------------------------------------------------------------------
#if 0
void DriveBase::OldFieldOrientedDrive()
{
    //define variables for the drive function
    float 			fXStick = 0.0;
    float 			fYStick = 0.0;
    float			fRotate = 0.0;
    bool			bAllowRotate = false;


    //get values from the controller
    fXStick = pRobot->DriverCmd.fMoveForward();
    fYStick = pRobot->DriverCmd.fMoveSideways();
    fRotate = pRobot->DriverCmd.fRotate();


    // deterime when to start turns based on d-pad
    if((pRobot->DriverCmd.POV() > -1) && (pRobot->Nav.bPresetTurning == false))
    {
    	pRobot->Nav.fGyroCommandYaw = pRobot->DriverCmd.POV();
    	pRobot->Nav.bPresetTurning = true;
    }
    //determine when to end the gyro turn
    if(fabs(pRobot->Nav.GetYawError())<10)
    {
    	pRobot->Nav.bPresetTurning = false;
    }

    //if((pRobot->Xbox1.GetX(frc::XboxController::kRightHand)>0.05) || (pRobot->Xbox1.GetX(frc::XboxController::kRightHand) < -0.05))
    //{
    //    bAllowRotate = true;
    //}
    //determine if the robot is allowed to rotate
    frc::SmartDashboard::PutNumber("X-Axis", pRobot->DriverCmd.fRotate());
    bAllowRotate = pRobot->DriverCmd.bManualRotate();

    //determine whether to get rotate values from gyro or controller
    if (bAllowRotate)
	{
        fRotate = pRobot->DriverCmd.fRotate();
        if (fRotate >  0.0) fRotate -= 0.05;
        if (fRotate <  0.0) fRotate += 0.05;
        if (fRotate >  0.5) fRotate  =  0.5;
        if (fRotate < -0.5) fRotate  = -0.5;

        pRobot->Nav.SetCommandYawToCurrent();
	}
    else
    {
        // Calculate a rotation rate from robot angle error
    	fRotate = pRobot->Nav.GetRotate();
    }

    //plug values into drive function
    pRobot->RobotDrive.DriveCartesian(fXStick, fYStick, fRotate, -(pRobot->Nav.GetYaw()));

    //reset gyro
    if(pRobot->DriverCmd.bResetGyro())
    {
        pRobot->Nav.ResetYaw();
    }
} // end FieldOrientedDrive()

#endif
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
    float 		fXStick;
    float 		fYStick;
    float		fRotate;
    bool		bAllowRotate = false;
    bool        bVisionTracked;
    float       fVisionTrackErrorX, fVisionTrackErrorY;
    float       fVisionTargetSizeX, fVisionTargetSizeY;
    char        szVisionMsg[100];

    // Get drive inputs
    fXStick = pRobot->DriverCmd.fMoveForward();
    fYStick = pRobot->DriverCmd.fMoveSideways();

    // Get vision track status and track errors
    pRobot->CameraTrk.GetTrackError(&bVisionTracked, &fVisionTrackErrorX, &fVisionTrackErrorY, &fVisionTargetSizeX, &fVisionTargetSizeY);

    // Vision track mode
    if (pRobot->DriverCmd.bTestButton(0))
        if (bVisionTracked)
            {
            fRotate = fVisionTrackErrorX * 0.5;
            pRobot->Nav.SetCommandYawToCurrent();
            sprintf(szVisionMsg, "TRACK    %4.1f", fRotate);
            }
        else
            {
            fRotate = pRobot->Nav.GetRotate();
            sprintf(szVisionMsg, "NO TRACK %4.1f", fRotate);
            } // end if vision track

    // Non-vision track mode
    else 
        {
        // Manual rotate
        if(pRobot->DriverCmd.bManualRotate())
            {
            fRotate = pRobot->DriverCmd.fRotate();
            pRobot->Nav.SetCommandYawToCurrent();
            sprintf(szVisionMsg, "MANUAL  %4.1f", fRotate);
            }
        // Gyro control rotate
        else
            {
            fRotate = pRobot->Nav.GetRotate();
            sprintf(szVisionMsg, "GYRO    %4.1f", fRotate);
            }
        } // end if not vision track

    frc::SmartDashboard::PutString("Vision Track", szVisionMsg);

    // Drive the robot
//    printf(" %s %5.2f\n", bVisionTracked?"TRACK   ":"NO TRACK", fRotate);
    pRobot->RobotDrive.DriveCartesian(fXStick, fYStick, fRotate, 0.0);

    } // end DriveToTarget()


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

float DriveBase::LimitFWDDrive(float InitDrive, bool Auto, float CommandDistance)
{

    float Distance = Ultra.GetRangeInches();
    SmartDashboard::PutNumber("Distance", Distance);
    
    float Error = Distance - CommandDistance;

    if(!(pRobot->DriverCmd.UltrasonicAllowed()) && !Auto)
    {
        return InitDrive;
    }
    float MaxSpeed = (Error / 100) +.05;
    SmartDashboard::PutNumber("Max Speed", MaxSpeed);
    return MaxSpeed;
}


// ----------------------------------------------------------------------------

void DriveBase::Init()
{
    Ultra.SetAutomaticMode(true);
}
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
    float position = pRobot->MotorControl_LR.GetSelectedSensorPosition();
    SmartDashboard::PutNumber("Position", position);
    pRobot->MotorControl_LR.Set(ControlMode::Position, 0);
    //pRobot->MotorControl_LR.Set(ControlMode::PercentOutput, pRobot->DriverCmd.fTestValue(0));
}