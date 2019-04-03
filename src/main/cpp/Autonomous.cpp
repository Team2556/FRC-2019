/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Autonomous.h"

Autonomous::Autonomous(Robot * pRobot, DriveBase * MecDrive, Elevator * ControlElevator, TeleopControl  * TeleopAuto) 
{
    this->pRobot = pRobot;
    this->MecDrive = MecDrive;
    this->ControlElevator = ControlElevator;
    this->TeleopAuto = TeleopAuto;
}

void Autonomous::AutoTeleopInit()
{
    AutoNumber = 0;
}

void Autonomous::AutoTeleop()
{
    TeleopAuto->TeleopMain();
}


void Autonomous::Auto1()
{
    float fForward = 0.0;
    float fStrafe  = 0.0;
    float fRotate  = 0.0;



    /*
        Action Number will increment by 10 to allow for new ones in between current ones

        10 - Drive Off of platform

        20 - turn to 61.25 deg to face left of rocket

        50 - drive forward with the ultrasonic until the line is detected

        60 - line up on the rocket and place the hatch
 
        70 - back away from the rocket

        80 - 

        90 - turn to face the player station

        100 - drive forward until the line follower and ultrasonic are lined up on the player station
    */

    switch (ActionNum)
    {
        case 10: // Drive Off of platform

            // Drive Commands
            FieldOrientedDrive = false;
            fForward = (AutoCounter - SectionStart) * .005;
            fStrafe = 0;

            #ifdef AUTO_ELE_ENABLED

            // Elevator Commands
            ControlElevator->ElevatorTilt(true); // keep the elevator tilted backwards as you drive off of the hab
            

            #endif
            // end section statement
            if (AutoCounter - SectionStart >= 90)
            {
                SectionStart = AutoCounter;
                ActionNum = 15;
            }
        break;

        case 15: // delay before turning
            fForward = 0.0;
            fStrafe = 0.0;
            //end section statement
            if (AutoCounter - SectionStart >= 20)
            {
                SectionStart = AutoCounter;
                ActionNum = 20;
            }
        break;

        case 20: // turn to -45 deg to face corner of habitat
            fForward = 0;
            fStrafe = 0;
            
            pRobot->Nav.SetCommandYaw(-45);

            // end section statement
            if (!pRobot->Nav.GetPresetTurning())
            {
                ActionNum = 30;
                SectionStart = AutoCounter;
            }
        break;

        case 30: // drive forward onto the carpet to be in line with the rocket
            fForward = .25;
            fStrafe = 0;


            if (AutoCounter - SectionStart >= 95)
            {
                SectionStart = AutoCounter;
                ActionNum = 40;
            }
        break;

        case 40: // turn to 61.25 deg to face left of rocket
            fForward = 0;
            fStrafe = 0;
            
            pRobot->Nav.SetCommandYaw(-28.75);


            // end section statement
            if (!pRobot->Nav.GetPresetTurning())
            {
                ActionNum = 50;
                SectionStart = AutoCounter;
            }

        break;

        case 50: // drive forward with the ultrasonic unitl the line is found
            fForward = MecDrive->LimitFWDDrive(14);
            fStrafe = pRobot->DriverCmd.GetAutoStrafe();


            #ifdef AUTO_ELE_ENABLED

            ControlElevator->ElevatorTilt(false); //tilt the elevator forward
            ControlElevator->ElevatorControl(DriverCommands::ElevatorHeight::Middle, DriverCommands::ElevatorMode::Hatch, 0);
            ControlElevator->WristControl(DriverCommands::ElevatorHeight::Middle, DriverCommands::ElevatorMode::Hatch);
            


            #endif

            if (pRobot->LineTracker.FrontSensors.bLineFound)
            {
                ActionNum = 60;
                SectionStart = AutoCounter;
            }        
        break;

        case 60: // Line up on the rocket and place the hatch
            static int HatchOutCounter = 0;
            fForward = MecDrive->LimitFWDDrive(20);
            fStrafe = pRobot->LineTracker.GetStrafe(0);;

            // the hatch will be placed on the rocket here
            #ifdef AUTO_ELE_ENABLED

            if(pRobot->LineTracker.GetStrafe(0) <.1 && ControlElevator->ElevatorControl(DriverCommands::ElevatorHeight::Middle, DriverCommands::ElevatorMode::Hatch, 0))
            {
                ControlElevator->RollerPistons(true);
                HatchOutCounter++;
            }

            #endif

            // in the end this will be for when the hatch is placed
            if(HatchOutCounter>=3)
            {
                ActionNum = 70;
                SectionStart = AutoCounter;
            }            
        break;

        case 70: // back away from the rocket
            FieldOrientedDrive = true; // re enable field oriented drive to make the motion back more smooth
            fForward = -.5;
            fStrafe = 0.0;

            #ifdef AUTO_ELE_ENABLED

            ControlElevator->ElevatorControl(DriverCommands::ElevatorHeight::Pickup, DriverCommands::ElevatorMode::Hatch, 0);

            #endif

            if (AutoCounter - SectionStart >= 20)
            {
                SectionStart = AutoCounter;
                ActionNum = 80;
            }
        break;

        case 80: // turn to face the player station and begin to move torwards it
            fForward = -.75;
            fStrafe = -.2;

            pRobot->Nav.SetCommandYaw(180);

            #ifdef AUTO_ELE_ENABLED

            ControlElevator->ElevatorControl(DriverCommands::ElevatorHeight::Pickup, DriverCommands::ElevatorMode::Hatch, 0);

            #endif

            // end section statement
            if (!pRobot->Nav.GetPresetTurning())
            {
                ActionNum = 90;
                SectionStart = AutoCounter;
            }
        break;

        case 90: // drive forward until the line follower and ultrasonic are lined up on the player station
            FieldOrientedDrive = false;
            fForward = MecDrive->LimitFWDDrive(10);
            fStrafe = pRobot->DriverCmd.GetAutoStrafe();

            #ifdef AUTO_ELE_ENABLED

            ControlElevator->ElevatorControl(DriverCommands::ElevatorHeight::Pickup, DriverCommands::ElevatorMode::Hatch, 0);

            #endif


            if (pRobot->LineTracker.FrontSensors.bLineFound)
            {
                ActionNum = 100;
                SectionStart = AutoCounter;
            }  
        break;

        case 100: // Line up on the player station
            fForward = MecDrive->LimitFWDDrive(20);
            fStrafe = pRobot->LineTracker.GetStrafe(0);

            // the hatch will be picked up here

            // in the end this will be for when the hatch is placed
            if(pRobot->DriverCmd.bTestButton(0))
            {
                ActionNum = -1;
                SectionStart = AutoCounter;
            } 
        break;

        case -1: default:
        fForward = 0;
        fStrafe = 0;

        break;
    }  


    if (AutoCounter == 0)
    {
        pRobot->Nav.ResetYaw();
    }
    fRotate = pRobot->Nav.GetRotate();
    
    if (!FieldOrientedDrive)
    {
        MecDrive->pRobot->RobotDrive.DriveCartesian(fStrafe, fForward, fRotate, 0.0); 
    }
    else
    {
        MecDrive->pRobot->RobotDrive.DriveCartesian(fStrafe, fForward, fRotate, -(pRobot->Nav.GetYaw())); 
    }

    SmartDashboard::PutNumber("Auto Section", ActionNum);
    AutoCounter++;
}

void Autonomous::Auto1Init()
{
    AutoNumber = 1;
    ActionNum  = 10;
    AutoCounter = 0;
    SectionStart = 0;
}

void Autonomous::Auto2()
{
    float fForward = 0.0;
    float fStrafe  = 0.0;
    float fRotate  = 0.0;



    /*
        Action Number will increment by 10 to allow for new ones in between current ones

        10 - Drive Off of platform

        20 - turn to 61.25 deg to face left of rocket

        50 - drive forward with the ultrasonic until the line is detected

        60 - line up on the rocket and place the hatch
 
        70 - back away from the rocket

        80 - 

        90 - turn to face the player station

        100 - drive forward until the line follower and ultrasonic are lined up on the player station
    */

    switch (ActionNum)
    {
        case 10: // Drive Off of platform
            FieldOrientedDrive = true;
            fForward = (AutoCounter - SectionStart) * .01;
            fStrafe = 0;

            // end section statement
            if (AutoCounter - SectionStart >= 90)
            {
                SectionStart = AutoCounter;
                ActionNum = 20;
            }
        break;

        case 20: // Drive Back toward Cargo Ship
            fForward = .65;
            fStrafe = 0;

            // end section statement
            /*if (MecDrive->SideUltra(30))
            {
                SectionStart = AutoCounter;
                ActionNum = 30;
            }*/
        break;

        case 30: // Turn toward the side face of the Rocket
            FieldOrientedDrive = true;
            fForward = .8;
            fStrafe  = -.6;
            pRobot->Nav.SetCommandYaw(-151.25);

            if (AutoCounter - SectionStart >= 90)
            {
                SectionStart = AutoCounter;
                ActionNum = -1;
            }
        break;

        case 40: // Drive toward Rocket line
        break;

        case 50: // Start line and gyro function
        break;

        case 60: // Back up and turn toward HPS      
        break;

        case 70: // Drive toward the HPS          
        break;

        case 80: // Turn around from HPS
        break;

        case 90: // Drive toward the other Rocket Side
  
        break;

        case 100: // Start Line Following and UltraSonic Function again
        break;  

        case -1: default:
        fForward = 0.0;
        fStrafe = 0.0;
        break;
    }  


    if (AutoCounter == 0)
    {

        pRobot->Nav.ResetYaw();
        SmartDashboard::PutBoolean("Gyro Reset", true);
    }
    fRotate = pRobot->Nav.GetRotate();
    
    if (!FieldOrientedDrive)
    {
        MecDrive->pRobot->RobotDrive.DriveCartesian(fStrafe, fForward, fRotate, 0.0); 
    }
    else
    {
        MecDrive->pRobot->RobotDrive.DriveCartesian(fStrafe, fForward, fRotate, -pRobot->Nav.GetYaw()); 
    }

    SmartDashboard::PutNumber("Auto Section", ActionNum);
    AutoCounter++;
}

void Autonomous::Auto2Init()
{
    AutoNumber = 2;
    ActionNum  = 30;
    AutoCounter = 0;
    SectionStart = 0;
    SmartDashboard::PutBoolean("Gyro Reset", false);
}

bool Autonomous::RollersSet()
{
    static int EncoderValue = -1;
    static bool EncoderDown = false;

    if(ControlElevator->ElevatorUpDown.GetSelectedSensorPosition() == 0 && EncoderDown == true)
    {
        return false;
    }
    else if(EncoderDown == false && ControlElevator->ElevatorUpDown.GetSelectedSensorPosition() < -50)
    {
        EncoderDown = true;
    }
    else
    {
        ControlElevator->ElevatorUpDown.Set(ControlMode::PercentOutput, -0.25);
        ControlElevator->ElevatorUpDownB.Follow(ControlElevator->ElevatorUpDown);
    }
}

void Autonomous::AutoInit()
{
}

void Autonomous::Auto()
{
    switch (AutoNumber)
    {
        case 0: default:
        AutoTeleop();
        break;
        case 1:
        Auto1();
        break;
        case 2:
        Auto2();
        break;
    }
}