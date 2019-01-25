/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "DriverCommands.h"

// ----------------------------------------------------------------------------
// Constructor / Destructor
// ----------------------------------------------------------------------------

DriverCommands::DriverCommands() 
    {
    CurrDriveMode = DriveMode::Gyro;
    }


// ----------------------------------------------------------------------------
// Moving commands
// ----------------------------------------------------------------------------

float DriverCommands::fMoveForward()
    {
    return Xbox1.GetX(frc::XboxController::kLeftHand);
    }


// ----------------------------------------------------------------------------

float DriverCommands::fMoveSideways()
    {
    return Xbox1.GetY(frc::XboxController::kLeftHand) * -1.0;
    }


// ----------------------------------------------------------------------------

float DriverCommands::fRotate()
    {
    return Xbox1.GetX(frc::XboxController::kRightHand);
    }

// ----------------------------------------------------------------------------

// Return true if player wants to rotate robot manually
bool DriverCommands::bManualRotate()
    {
    switch (this->CurrDriveMode)
        {
        case DriveMode::Gyro :
            if ((this->fRotate() > 0.05) || (this->fRotate() < -0.05))
                return true;
            break;
        case DriveMode::FieldOriented :
            if (Xbox1.GetTriggerAxis(frc::XboxController::kRightHand) > 0.5)
                return true;
            break;
        default :
            return false;
            break;
        }
    }

// ----------------------------------------------------------------------------

// Return the currently commanded Point of View as an integer angle
// in degrees. Return -1 if no POV is commanded.
int DriverCommands::POV()
    {
    return Xbox1.GetPOV();
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

    return CurrDriveMode;
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
// Return true to command roller assembly down
bool DriverCommands::bRollersDown()
    {
    return Xbox2.GetYButton();
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
            bButtonValue = Xbox2.GetYButton();
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

    if ((iControl < 0) || (iControl > 1))
        return 0.0;

    switch (iControl)
        {
        case 0 :
            fControlValue = Xbox2.GetTriggerAxis(frc::XboxController::JoystickHand::kRightHand);
            break;
        case 1 :
            fControlValue = Xbox2.GetTriggerAxis(frc::XboxController::JoystickHand::kLeftHand);
            break;
        default :
            fControlValue = 0.0;
            break;
        }

    return fControlValue;
    }
