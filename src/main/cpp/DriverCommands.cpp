/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// http://first.wpi.edu/FRC/roborio/release/docs/cpp/classfrc_1_1XboxController.html
// http://first.wpi.edu/FRC/roborio/release/docs/cpp/classfrc_1_1Joystick.html

#include "DriverCommands.h"

#define STICK_ROTATE_MODE
#define STICK_ROTATE_THRESHOLD        0.2

// ----------------------------------------------------------------------------
// Constructor / Destructor
// ----------------------------------------------------------------------------

DriverCommands::DriverCommands() 
{
    CurrDriveMode = DriveMode::Gyro;
    CMDElevatorHeight = ElevatorHeight::Middle;
    frc::SmartDashboard::PutString("Height", "Middle");
    CMDElevatorMode = ElevatorMode::Hatch;
    frc::SmartDashboard::PutString("Mode", "Hatch");
}


// ----------------------------------------------------------------------------
// General Commands
// ----------------------------------------------------------------------------

bool DriverCommands::GetClimbMode()
{
    if (Xbox1.GetStartButton())
    {
        ClimbMode = false;
    }
    if (Xbox1.GetBackButton())
    {
        ClimbMode = true;
    }
    return ClimbMode;
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
    float fRotate = JStick1.GetTwist();
#ifdef STICK_ROTATE_MODE
    if (fRotate > 0.0) fRotate -= STICK_ROTATE_THRESHOLD;
    if (fRotate < 0.0) fRotate += STICK_ROTATE_THRESHOLD;
#endif
    return fRotate;
#else
    return Xbox1.GetX(frc::XboxController::kRightHand);
#endif
    }

// ----------------------------------------------------------------------------

// Return true if player wants to rotate robot manually
bool DriverCommands::bManualRotate()
    {
#ifdef JOYSTICK
#ifdef STICK_ROTATE_MODE
    if ((fRotate() > STICK_ROTATE_THRESHOLD) || (fRotate() < -STICK_ROTATE_THRESHOLD))
        return true;
    else
        return false;
#else
        return JStick1.GetTrigger();
#endif

#else
        if ((this->fRotate() > 0.05) || (this->fRotate() < -0.05))
        {
            return true;
        }
        else
        {
            return false;
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
#ifdef JOYSTICK
#else
    return Xbox1.GetAButton();
#endif
    }


// ----------------------------------------------------------------------------

// Check for changes to current drive mode and then return it
DriverCommands::DriveMode DriverCommands::GetDriveMode()
    {
#ifdef JOYSTICK
   if (JStick1.GetRawButton(5))
        CurrDriveMode = DriveMode::Gyro;

   if (JStick1.GetRawButton(6))
        CurrDriveMode = DriveMode::DriveToTarget;

#else
   if (Xbox1.GetBButtonPressed())
        CurrDriveMode = DriveMode::Gyro;

   if (Xbox1.GetXButtonPressed())
        CurrDriveMode = DriveMode::FieldOriented;
   if (Xbox1.GetYButtonPressed())
        CurrDriveMode = DriveMode::Normal;
#endif

    return CurrDriveMode;
    }

// ----------------------------------------------------------------------------
bool DriverCommands::GetLineUp()
{
    if(Xbox1.GetBumper(frc::XboxController::kLeftHand))
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
    if (Xbox1.GetTriggerAxis(frc::XboxController::kRightHand) > Xbox2.GetTriggerAxis(frc::XboxController::kLeftHand))
    {
        return Xbox2.GetTriggerAxis(frc::XboxController::kRightHand);
    }
    return -Xbox1.GetTriggerAxis(frc::XboxController::kLeftHand);
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

    return fUpDownSpeed/2;
    }


// ----------------------------------------------------------------------------
// Return true to command roller assembly down or a false to go up
bool DriverCommands::bRollersDown()
{
    bool        RollersDown = false; // true if the rollers/wrist are down
    
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
    if (Xbox2.GetYButtonPressed())
    {
        rollerBool = !rollerBool;
    }

    return ElevatorTilted;
}


bool DriverCommands::Outtake()
{
    return Xbox2.GetYButtonPressed();
}

bool DriverCommands::Intake()
{
    return Xbox2.GetAButton();
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


void DriverCommands::HeightIntEnum() // converts the height int to the height enum
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

        case 3 : // Cargo Ship
            CMDElevatorHeight = DriverCommands::ElevatorHeight::CargoShip;
            frc::SmartDashboard::PutString("Height", "Cargo Ship");
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
        if (POV == 0)
        {
            iElevatorHeight  = 0;
            HeightChanged = true;
        }
    }
    else if (iElevatorHeight >= 0 && iElevatorHeight <= 3 && !HeightChanged) // ensures the elevator isnt in pickup mode
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
        else if (iElevatorHeight > 3)
        {
            iElevatorHeight = 3;
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

bool DriverCommands::bAutomaticElevator()
{
    return Xbox2.GetBumper(frc::XboxController::kLeftHand);
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

#ifdef JOYSTICK
    switch (iButton)
        {
        case 0  : bButtonValue = JStick1.GetRawButton( 2); break;   // Thumb
        case 1  : bButtonValue = JStick1.GetRawButton(11); break;
        case 2  : bButtonValue = JStick1.GetRawButton(12); break;
        case 3  : bButtonValue = JStick1.GetRawButton( 9); break;
        case 4  : bButtonValue = JStick1.GetRawButton(10); break;
        case 5  : bButtonValue = JStick1.GetRawButton( 7); break;
        default : bButtonValue = false;                    break;
        }

#else
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
            bButtonValue = Xbox1.GetBumper(frc::XboxController::JoystickHand::kRightHand);
            break;
        default :
            bButtonValue = false;
            break;
        }
#endif

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
