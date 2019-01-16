/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "DriveBase.h"

//Constructor
DriveBase::DriveBase(Robot * pRobot) 
{
    this->pRobot = pRobot;
    drivemode = 0;
}


// All comments are in field oriented drive

void DriveBase::GyroDrive()
{
    float 			fXStick = 0.0;
    float 			fYStick = 0.0;
    float			fRotate = 0.0;
    bool			bAllowRotate = false;


    fXStick = pRobot->Xbox1.GetX(frc::XboxController::kLeftHand);
    fYStick = (pRobot->Xbox1.GetY(frc::XboxController::kLeftHand) * -1.0);
    fRotate = pRobot->Xbox1.GetX(frc::XboxController::kRightHand);



    if(pRobot->Xbox1.GetPOV()>-1 && pRobot->pNavGyro.bPresetTurning == false)
    {
    	pRobot->pNavGyro.fGyroCommandYaw = pRobot->pNavGyro.fGyroCommandYaw + pRobot->Xbox1.GetPOV();
    	pRobot->pNavGyro.bPresetTurning = true;
    }
    if(fabs(pRobot->pNavGyro.GetYawError())<10)
    {
    	pRobot->pNavGyro.bPresetTurning = false;
    }

    bAllowRotate = pRobot->Xbox1.GetTriggerAxis(frc::XboxController::kRightHand)>.5;

    if (bAllowRotate)
	{
	fRotate = pRobot->Xbox1.GetX(frc::XboxController::kRightHand);
	fRotate = pRobot->pNavGyro.CorrectRotate(fRotate);
	pRobot->pNavGyro.SetCommandYawToCurrent();
	}
    else
    {
        // Calculate a rotation rate from robot angle error
    	fRotate = pRobot->pNavGyro.GetRotate();
    }

    pRobot->RobotDrive.DriveCartesian(fXStick, fYStick, fRotate, 0.0);



}

void DriveBase::NormalDrive()
{
    pRobot->RobotDrive.DriveCartesian(
        pRobot->Xbox1.GetX(frc::XboxController::kLeftHand), 
        pRobot->Xbox1.GetY(frc::XboxController::kLeftHand)*-1, 
        pRobot->Xbox1.GetX(frc::XboxController::kRightHand));
}

void DriveBase::FieldOrientedDrive()
{
    //define variables for the drive function
    float 			fXStick = 0.0;
    float 			fYStick = 0.0;
    float			fRotate = 0.0;
    bool			bAllowRotate = false;


    //get values from the controller
    fXStick = pRobot->Xbox1.GetX(frc::XboxController::kLeftHand);
    fYStick = (pRobot->Xbox1.GetY(frc::XboxController::kLeftHand) * -1.0);
    fRotate = pRobot->Xbox1.GetX(frc::XboxController::kRightHand);


    // deterime when to start turns based on d-pad
    if(pRobot->Xbox1.GetPOV()>-1 && pRobot->pNavGyro.bPresetTurning == false)
    {
    	pRobot->pNavGyro.fGyroCommandYaw = pRobot->Xbox1.GetPOV();
    	pRobot->pNavGyro.bPresetTurning = true;
    }
    //determine when to end the gyro turn
    if(fabs(pRobot->pNavGyro.GetYawError())<10)
    {
    	pRobot->pNavGyro.bPresetTurning = false;
    }

    //if((pRobot->Xbox1.GetX(frc::XboxController::kRightHand)>0.05) || (pRobot->Xbox1.GetX(frc::XboxController::kRightHand) < -0.05))
    //{
    //    bAllowRotate = true;
    //}
    //determine if the robot is allowed to rotate
    SmartDashboard::PutNumber("X-Axis", pRobot->Xbox1.GetX(frc::XboxController::kRightHand));
    bAllowRotate = pRobot->Xbox1.GetTriggerAxis(frc::XboxController::kRightHand)>.5;

    //determine whether to get rotate values from gyro or controller
    if (bAllowRotate)
	{
        fRotate = pRobot->Xbox1.GetX(frc::XboxController::kRightHand);
        if(fRotate > 0) 
        {
            fRotate = fRotate - 0.05;
        }
        if(fRotate < 0) 
        {
            fRotate = fRotate + 0.05;
        }
        fRotate = pRobot->pNavGyro.CorrectRotate(fRotate);
        pRobot->pNavGyro.SetCommandYawToCurrent();
	}
    else
    {
        // Calculate a rotation rate from robot angle error
    	fRotate = pRobot->pNavGyro.GetRotate();
    }

    //plug values into drive function
    pRobot->RobotDrive.DriveCartesian(fXStick, fYStick, fRotate, -(pRobot->pNavGyro.pNavX->GetYaw()));

    //reset gyro
    if(pRobot->Xbox1.GetAButton())
    {
        pRobot->pNavGyro.ResetYaw();
    }
}




void DriveBase::GyroTurningDrive()
{
    float 			fXStick = 0.0;
    float 			fYStick = 0.0;
    float			fRotate = 0.0;
    bool		bAllowRotate = false;
    

    fXStick = pRobot->Xbox1.GetX(frc::XboxController::kLeftHand);
    fYStick = (pRobot->Xbox1.GetY(frc::XboxController::kLeftHand) * -1.0);


    if((pRobot->Xbox1.GetX(frc::XboxController::kRightHand)>0.05) || (pRobot->Xbox1.GetX(frc::XboxController::kRightHand) < -0.05))
    {
        bAllowRotate = true;
    }

    if(pRobot->Xbox1.GetPOV()>-1 && pRobot->pNavGyro.bPresetTurning == false)
    {
    	pRobot->pNavGyro.fGyroCommandYaw = pRobot->Xbox1.GetPOV();
    	pRobot->pNavGyro.bPresetTurning = true;
        bAllowRotate = false;
    }
    if(fabs(pRobot->pNavGyro.GetYawError())<10)
    {
    	pRobot->pNavGyro.bPresetTurning = false;
    }
    else if(pRobot->pNavGyro.bPresetTurning == true)
    {
        bAllowRotate = false;
    }






    

    if(bAllowRotate == true)
    {
        bRotatePrevious = true;
        
    }


    if(bAllowRotate == false && bRotatePrevious == true && stopHoldCounter < 5)
    {
        stopHoldCounter++;
    }
    else if(bAllowRotate == false && bRotatePrevious == true && stopHoldCounter >= 5)
    {
        stopHoldCounter = 0;
        bRotatePrevious = false;
    }


    if (bRotatePrevious)
	{
        fRotate = pRobot->Xbox1.GetX(frc::XboxController::kRightHand);
        if(fRotate > 0) 
        {
            fRotate = fRotate - 0.05;
        }
        if(fRotate < 0) 
        {
            fRotate = fRotate + 0.05;
        }
        fRotate = pRobot->pNavGyro.CorrectRotate(fRotate);
        pRobot->pNavGyro.SetCommandYawToCurrent();
        SmartDashboard::PutBoolean("Gryo Enabled", false);

	}
    else
    {
        // Calculate a rotation rate from robot angle error
    	fRotate = pRobot->pNavGyro.GetRotate();
        SmartDashboard::PutBoolean("Gryo Enabled", true);
    }
    SmartDashboard::PutNumber("Counter", stopHoldCounter);
    SmartDashboard::PutBoolean("Allow Rotate", bAllowRotate);
    SmartDashboard::PutBoolean("Rotate Previous", bRotatePrevious);

    pRobot->RobotDrive.DriveCartesian(fXStick, fYStick, fRotate, -(pRobot->pNavGyro.pNavX->GetYaw()));


    if(pRobot->Xbox1.GetAButton())
    {
        pRobot->pNavGyro.ResetYaw();
    }
}




void DriveBase::Drive()
{
    if(pRobot->Xbox1.GetBButton())
    {
      this->drivemode = true;
    }
    else if(pRobot->Xbox1.GetXButton())
    {
      this->drivemode = false;
    }
    if (this->drivemode == true)
    {
      this->GyroTurningDrive();
      SmartDashboard::PutString("DriveMode", "Gryo");
    }
    else
    {
      this->FieldOrientedDrive();
      SmartDashboard::PutString("DriveMode", "Field Orienteds");
    }
}