/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// http://first.wpi.edu/FRC/roborio/release/docs/cpp/classfrc_1_1XboxController.html
// http://first.wpi.edu/FRC/roborio/release/docs/cpp/classfrc_1_1Joystick.html

#include "DriverCommands.h"

// ----------------------------------------------------------------------------
// Constructor / Destructor
// ----------------------------------------------------------------------------

DriverCommands::DriverCommands() 
{
    CurrDriveMode = DriveMode::Gyro;
    CMDElevatorHeight = ElevatorHeight::Low;
    frc::SmartDashboard::PutString("Height", "Low");
    CMDElevatorMode = ElevatorMode::Hatch;
    frc::SmartDashboard::PutString("Mode", "Hatch");
}


// ----------------------------------------------------------------------------
// Moving commands
// ----------------------------------------------------------------------------

float DriverCommands::fMoveForward()
    {
#ifdef JOYSTICK
    return JStick1.GetX();
#else
    return Xbox1.GetY(frc::XboxController::kLeftHand) * -1.0;
#endif
    }


// ----------------------------------------------------------------------------

float DriverCommands::fMoveSideways()
    {
    
#ifdef JOYSTICK
    return JStick1.GetY() * -1.0;
#else
    return Xbox1.GetX(frc::XboxController::kLeftHand);
#endif
    }


// ----------------------------------------------------------------------------

float DriverCommands::fRotate()
    {
#ifdef JOYSTICK
    return JStick1.GetTwist();
#else
    return Xbox1.GetX(frc::XboxController::kRightHand);
#endif
    }

// ----------------------------------------------------------------------------

// Return true if player wants to rotate robot manually
bool DriverCommands::bManualRotate()
    {
    frc::SmartDashboard::PutNumber("Right X", this->fRotate());
#ifdef JOYSTICK
    return JStick1.GetTrigger();
#else
    switch (this->CurrDriveMode)
        {
        case DriveMode::FieldOriented :
            if ((this->fRotate() > 0.05) || (this->fRotate() < -0.05))
                return true;
            break;
        case DriveMode::Gyro :
            if (Xbox1.GetTriggerAxis(frc::XboxController::kRightHand) > 0.5)
                return true;
            break;
        default :
            return false;
            break;
        }
#endif
    }

// ----------------------------------------------------------------------------

// Return the currently commanded Point of View as an integer angle
// in degrees. Return -1 if no POV is commanded.
int DriverCommands::POV()
    {
#ifdef JOYSTICK
    return JStick1.GetPOV();
#else
    return Xbox1.GetPOV();
#endif
    }


// ----------------------------------------------------------------------------

bool DriverCommands::bResetGyro()
    {
    return Xbox1.GetAButton();
    }


// ----------------------------------------------------------------------------

// Check for changes to current drive mode and then return it
DriverCommands::DriveMode DriverCommands::GetDriveMode()
    {
   if (Xbox1.GetBButtonPressed())
        CurrDriveMode = DriveMode::Gyro;

   if (Xbox1.GetXButtonPressed())
        CurrDriveMode = DriveMode::FieldOriented;
   if (Xbox1.GetYButtonPressed())
        CurrDriveMode = DriveMode::Normal;

    return CurrDriveMode;
    }

// ----------------------------------------------------------------------------
bool DriverCommands::GetLineUpStrafe()
{
    if(Xbox1.GetTriggerAxis(frc::XboxController::kLeftHand)> .5)
    {
        return true;
    }
    return false;
}

bool DriverCommands::UltrasonicAllowed()
{
    return Xbox1.GetBumper(frc::XboxController::kLeftHand);
}

double DriverCommands::GetAutoStrafe()
{
    if (Xbox2.GetTriggerAxis(frc::XboxController::kRightHand) > Xbox2.GetTriggerAxis(frc::XboxController::kLeftHand))
    {
        return Xbox2.GetTriggerAxis(frc::XboxController::kRightHand);
    }
    return -Xbox2.GetTriggerAxis(frc::XboxController::kLeftHand);
}


// ----------------------------------------------------------------------------
// Elevator commands
// ----------------------------------------------------------------------------

// Return elevator up/down speed
// From -1.0 to +1.0
float DriverCommands::fElevatorUpDownSpeed()
    {
    float   fUpDownSpeed;

    if (Xbox2.GetTriggerAxis(frc::XboxController::kRightHand) > Xbox2.GetTriggerAxis(frc::XboxController::kLeftHand))
        fUpDownSpeed =  Xbox2.GetTriggerAxis(frc::XboxController::kRightHand);
    else 
        fUpDownSpeed = -Xbox2.GetTriggerAxis(frc::XboxController::kLeftHand);

    return fUpDownSpeed;
    }


// ----------------------------------------------------------------------------
// Return true to command roller assembly down or a false to go up
bool DriverCommands::bRollersDown()
{
    if (Xbox2.GetBumper(frc::XboxController::kRightHand))
    {
        RollersDown = false;
    }
    if (Xbox2.GetBumper(frc::XboxController::kLeftHand))
    {
        RollersDown = true;
    }
    return RollersDown;
}

bool DriverCommands::bElevatorTilt()
{
    if (Xbox2.GetStartButton())
    {
        ElevatorTilted = false;
    }
    if (Xbox2.GetBackButton())
    {
        ElevatorTilted = true;
    }
    return ElevatorTilted;
}
bool DriverCommands::bRollerPistons()
{
    if (Xbox2.GetBButtonPressed())
    {
        rollerBool = !rollerBool;
    }

    return ElevatorTilted;
}

// ----------------------------------------------------------------------------

DriverCommands::ElevatorMode DriverCommands::GetElevatorMode()
{
    if (Xbox2.GetPOV() == 270)
    {
        CMDElevatorMode = Hatch;
        frc::SmartDashboard::PutString("Mode", "Hatch");
    }
    if (Xbox2.GetPOV() == 90)
    {
        CMDElevatorMode = Cargo;
        frc::SmartDashboard::PutString("Mode", "Cargo");
    }
    return CMDElevatorMode;
}


void DriverCommands::HeightIntEnum()
{
    switch (iElevatorHeight)
    {
        case 0 : // low
            CMDElevatorHeight = DriverCommands::ElevatorHeight::Low;
            frc::SmartDashboard::PutString("Height", "Low");
        break;

        case 1 : // Middle
            CMDElevatorHeight = DriverCommands::ElevatorHeight::Middle;
            frc::SmartDashboard::PutString("Height", "Middle");
        break;

        case 2 : // High
            CMDElevatorHeight = DriverCommands::ElevatorHeight::High;
            frc::SmartDashboard::PutString("Height", "High");
        break;

        case -2: // Ground Pickup
            CMDElevatorHeight =  DriverCommands::ElevatorHeight::GroundPickup;
            frc::SmartDashboard::PutString("Height", "Ground Pickup");
        break;

        case -1: // Pickup
            CMDElevatorHeight = DriverCommands::ElevatorHeight::Pickup;
            frc::SmartDashboard::PutString("Height", "Pickup");
        break;
    }
}

DriverCommands::ElevatorHeight DriverCommands::GetElevatorHeight()
{
    int POV = Xbox2.GetPOV();
    frc::SmartDashboard::PutNumber("POV", POV);
    static bool HeightChanged = false;
    if (POV == -1)
    {
        HeightChanged = false;
    }
    

    if (iElevatorHeight < 0)
    {
        printf("Here");
        if (POV == 0)
        {
            iElevatorHeight  = 0;
            HeightChanged = true;
        }
    }
    else if (iElevatorHeight >= 0 && iElevatorHeight <= 2 && !HeightChanged) // ensures the elevator isnt in pickup mode
    {
        if(POV == 0)
        {
            iElevatorHeight++;
            HeightChanged = true;
        }
        else if (POV == 180)
        {
            iElevatorHeight--;
            HeightChanged = true;
        }


        if (iElevatorHeight < 0)
        {
            iElevatorHeight = 0;
        }
        else if (iElevatorHeight > 2)
        {
            iElevatorHeight = 2;
        }
    }
    if(Xbox2.GetStickButtonPressed(frc::XboxController::kRightHand)) // when pressing the right stick down enable and disable pickup 
    {
         // enable pickup mode
        
        iElevatorHeight = -1;

    }
    if(Xbox2.GetStickButtonPressed(frc::XboxController::kLeftHand)) // when pressing the right stick down enable and disable ground pickup 
    {
         // enable pickup mode
        iElevatorHeight = -2;
    }
    frc::SmartDashboard::PutNumber("Height Number", iElevatorHeight);
    this->HeightIntEnum();
    return CMDElevatorHeight;
}


// ----------------------------------------------------------------------------
// Test commands
// ----------------------------------------------------------------------------

// Designate some buttons as test buttons just to be able to test
// some functionality.
bool DriverCommands::bTestButton(int iButton)
    {
    bool    bButtonValue;

    if ((iButton < 0) || (iButton > 5))
        return false;

    switch (iButton)
        {
        case 0 :
            bButtonValue = Xbox2.GetAButton();
            break;
        case 1 :
            bButtonValue = Xbox2.GetBButton();
            break;
        case 2 :
            bButtonValue = Xbox2.GetXButton();
            break;
        case 3 :
            bButtonValue = Xbox2.GetYButtonPressed();
            break;
        case 4 :
            bButtonValue = Xbox2.GetBumper(frc::XboxController::JoystickHand::kLeftHand);
            break;
        case 5 :
            bButtonValue = Xbox2.GetBumper(frc::XboxController::JoystickHand::kRightHand);
            break;
        default :
            bButtonValue = false;
            break;
        }

    return bButtonValue;
    }


// ----------------------------------------------------------------------------

// Return a control test value
// From -1.0 to +1.0
float DriverCommands::fTestValue(int iControl)
    {
    float   fControlValue;

    if ((iControl < 0) || (iControl > 3))
        return 0.0;

    switch (iControl)
        {
        case 0 :
            fControlValue = Xbox2.GetTriggerAxis(frc::XboxController::JoystickHand::kRightHand);
            break;
        case 1 :
            fControlValue = Xbox2.GetTriggerAxis(frc::XboxController::JoystickHand::kLeftHand);
            break;
        case 2 :
            fControlValue = Xbox2.GetX(frc::XboxController::kLeftHand);
            break;
        case 3 :
            fControlValue = Xbox2.GetY(frc::XboxController::kLeftHand);
            break;
        default :
            fControlValue = 0.0;
            break;
        }

    return fControlValue;
    }
