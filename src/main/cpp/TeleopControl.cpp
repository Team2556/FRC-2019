/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "TeleopControl.h"

TeleopControl::TeleopControl(Robot * pRobot, DriveBase * MecDrive, Elevator * ControlElevator, Climb * Climber) 
{
    this->pRobot = pRobot;
    this->MecDrive = MecDrive;
    this->ControlElevator = ControlElevator;
    this->Climber = Climber;
}



void TeleopControl::TeleopMain()
{
    if (!pRobot->DriverCmd.GetClimbMode())
    {
        this->TeleopDrive();
        pRobot->LineTracker.UpdateValues();
        ControlElevator->ElevatorControls();
        Climber->HoldIn();
    }
    else
    {
        Climber->Climbing();
    }
    frc::SmartDashboard::PutNumber("Angle", pRobot->Nav.GetYaw());
    frc::SmartDashboard::PutNumber("Distance", pRobot->UltraLF.GetRangeInches());
}

void TeleopControl::TeleopDrive()
{
    float   fForward = 0;
    float   fStrafe = 0;
    float   fRotate = 0;
    bool    bFOD = false; // Field oriented drive




    switch (pRobot->DriverCmd.GetDriveMode())
    {
        case DriverCommands::DriveMode::Gyro :
            frc::SmartDashboard::PutString("DriveMode", "Gyro");
            MecDrive->GyroDrive(&fForward, & fStrafe, &fRotate);
            break;

        case DriverCommands::DriveMode::FieldOriented :
            frc::SmartDashboard::PutString("DriveMode", "Field Oriented");
            MecDrive->FieldOrientedDrive(& fForward, &fStrafe, &fRotate, &bFOD);
            break;

        case DriverCommands::DriveMode::Normal :
        default :
            frc::SmartDashboard::PutString("DriveMode", "Normal Drive");
            MecDrive->NormalDrive(&fForward, &fStrafe, &fRotate);
            break;
    }



    if (pRobot->DriverCmd.GetLineUp())
    {
        SmartDashboard::PutBoolean("Auto Line Up", true);
        this->AutoLineUp(&fForward, &fStrafe, &fRotate);
        bFOD = false;
    }
    else 
    {
        this->AutoLineUpState = 0;
        SmartDashboard::PutBoolean("Auto Line Up", false);
    }



    MecDrive->Drive(fForward, fStrafe, fRotate, bFOD);

}

void TeleopControl::AutoLineUp(float * fForward, float * fStrafe, float *fRotate)
{
    static int  StopCounter = 0;
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

    
    if (!bVisionTracked && !pRobot->LineTracker.FrontSensors.bLineFound)
    {
        AutoLineUpState = 0;
    }


    switch (AutoLineUpState)
    {
        case 0 : // pre AutoLineUpState

            if (bVisionTracked)
            {
                AutoLineUpState = 10;
            }
            else if (pRobot->LineTracker.FrontSensors.bLineFound)
            {
                AutoLineUpState = 20;
            }

        break;

        case 10 :

            if (bVisionTracked) 
            {
                *fRotate = fVisionTrackErrorX * 0.35;
                pRobot->Nav.SetCommandYawToCurrent();
            }
            if (bDistanceGood)
            {
                *fForward = MecDrive->LimitFWDDrive(18);
            }

            if(pRobot->LineTracker.FrontSensors.bLineFound)
            {
                AutoLineUpState = 15;
            }
        break;

        case 15 :

            *fForward = -.1;
            *fStrafe = .05;

            StopCounter++;

            if (StopCounter >= 3)
            {
                AutoLineUpState = 17;
                StopCounter = 0;
            }
        break;

        case 17 :
            *fForward = 0;
            *fStrafe = 0;
            pRobot->Nav.SetCommandYaw(MecDrive->FindClose(pRobot->Nav.GetYaw()));

            if (fabs(pRobot->Nav.GetYawError()) < 4)
            {
                AutoLineUpState = 20;
            }
        break;

        case 20 : 
            *fForward = MecDrive->LimitFWDDrive(18);
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
                AutoLineUpState = 30;
                StopCounter = 0;
            }
        break;

        case 30 :
            *fForward = MecDrive->LimitFWDDrive(9);
            *fStrafe = 0;
        break;


    }
    SmartDashboard::PutNumber("AutoLineUpAutoLineUpState", AutoLineUpState);
}



