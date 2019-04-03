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
        pRobot->LineTracker.UpdateValues();
        this->TeleopDrive();
        this->TeleopElevator();
        Climber->HoldIn();
        frc::SmartDashboard::PutString("ClimbMode", "Disabled");
    }
    else
    {
        Climber->Climb2();
        //Climber->NewClimbing();
        
        this->TeleopDrive();
        ControlElevator->CMDMode = pRobot->DriverCmd.GetElevatorMode();
        frc::SmartDashboard::PutString("ClimbMode", "Enabled");
    }
    frc::SmartDashboard::PutNumber("Climb Encoder", Climber->ClimbMotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Pitch", pRobot->Nav.GetTilt());
    frc::SmartDashboard::PutNumber("Angle", pRobot->Nav.GetYaw());
    frc::SmartDashboard::PutNumber("Distance", pRobot->UltraLF.GetRangeInches());
    frc::SmartDashboard::PutNumber("Get Strafe", pRobot->LineTracker.GetStrafe(0));    
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

    SmartDashboard::PutNumber("AutoLineUpAutoLineUpState", AutoLineUpState);

    frc::SmartDashboard::PutNumber("Forward", fForward);
    frc::SmartDashboard::PutNumber("Strafe", fStrafe);
    frc::SmartDashboard::PutNumber("Rotate", fRotate);

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

    
    /*if (!bVisionTracked && !pRobot->LineTracker.FrontSensors.bLineFound)
    {
        AutoLineUpState = 0;
    }*/


    switch (AutoLineUpState)
    {
        case 0 : // pre AutoLineUpState

            if (pRobot->LineTracker.FrontSensors.bLineFound)
            {
                AutoLineUpState = 20;
            }
            else
            {
                AutoLineUpState = 10;
            }

        break;

        case 10 :
           if(bVisionTracked)
            {
                *fRotate = fVisionTrackErrorX * 0.35;
                pRobot->Nav.SetCommandYawToCurrent();
            }
            if (fDistanceToTarget > 11.3)
            {
                *fForward = MecDrive ->LimitFWDDrive(11); // find real distance at competition -- Houston
            
            }
            if(pRobot->LineTracker.FrontSensors.bLineFound)
            {
                AutoLineUpState = 15;
            }
            if(fDistanceToTarget < 10.9)
            {
                AutoLineUpState = 0;
            }
        break;

        case 15 :

            *fForward = -.05;
            *fStrafe = .001;
            

            StopCounter++;

            if (StopCounter >= 2.5)
            {
                AutoLineUpState = 17;
                StopCounter = 0;
            }
        break;

        case 17 :
            AutoLineUpState = 20;
        break;

        case 20 : 
            *fForward = 0;
            *fStrafe = 0;
            pRobot->Nav.SetCommandYaw( MecDrive->FindClose( pRobot->Nav.GetYaw()));


            *fForward = MecDrive->LimitFWDDrive(15); // ** might cause the robot to drive into the rocket
            *fStrafe = pRobot->LineTracker.GetStrafe(*fStrafe);

            
            

            if (pRobot->LineTracker.GetStrafe(*fStrafe) == 0 && pRobot->LineTracker.FrontSensors.bLineFound)
            {
                StopCounter++;
            }
            else 
            {
                StopCounter = 0;
            }

            if (StopCounter >= 5 && fabs(pRobot->Nav.GetYawError()) < 4)
            {
                AutoLineUpState = 25;
                StopCounter = 0;
            }
        break;

        case 25 :
            
            
            if (ControlElevator->ElevatorInPos(pRobot->DriverCmd.GetElevatorHeight(), pRobot->DriverCmd.GetElevatorMode()))
            {
                AutoLineUpState = 30;
            }
        break;

        case 30 :
            *fForward = MecDrive->LimitFWDDrive(9);
            *fStrafe = 0;

            if (false) // navx accelerometer
            {
                AutoLineUpState = 40;
            }
        break;

        case 40 :
            if (pRobot->DriverCmd.GetElevatorMode() == DriverCommands::ElevatorMode::Hatch)
            {
                ControlElevator->RollerPistons(true); // extend the pistons
            }
            else if (pRobot->DriverCmd.GetElevatorMode() == DriverCommands::ElevatorMode::Cargo)
            {
                ControlElevator->RollerOut();
                
            }
            StopCounter++;

            if (StopCounter >= 5)
            {
                AutoLineUpState = -1;
                StopCounter = 0;
            }
        break;

        case 50 :
            *fForward = MecDrive->LimitFWDDrive(19);


        break;

        case -1:
        default:
            *fForward = 0;
            *fStrafe = 0;
        break;
    }
    
}

void TeleopControl::TeleopElevator()
{
    if (pRobot->DriverCmd.GetLineUp())
    {
        if (!(AutoLineUpState > 25))
        {
            ControlElevator->ElevatorControl(DriverCommands::ElevatorHeight::Low, DriverCommands::ElevatorMode::Hatch, true);
        }
        else
        {
            ControlElevator->ElevatorControl(pRobot->DriverCmd.GetElevatorHeight(), pRobot->DriverCmd.GetElevatorMode(), true);
        }
        
    }
    else
    {
        ControlElevator->ElevatorControls();
    }
}