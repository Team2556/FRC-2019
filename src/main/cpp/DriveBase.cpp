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

    pRobot->RobotDrive.DriveCartesian(fXStick, fYStick, fRotate, -(pRobot->pNavGyro.pNavX->GetYaw()));


    if(pRobot->Xbox1.GetAButton())
    {
        pRobot->pNavGyro.ResetYaw();
    }
 




}
