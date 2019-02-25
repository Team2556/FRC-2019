/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>

#include "LineTrack.h"
#include "Robot.h"

// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------

ColorSens::ColorSens() 
{
}

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------

void ColorSens::UpdateValues()
{
    // Read the values and see if a line was found
    FrontSensors.bLineFound = false;
    for(int i = 0; i < LT_SENSORS_FRONT; i++)
    {
        FrontSensors.Values[i] = RLSens[i].Get();
        FrontSensors.Values[3] = false; // take out when color sensor is fixed
        if (FrontSensors.Values[i] == true)
            FrontSensors.bLineFound = true;
    }

    // Find the minimum index value that is true
    for (int i = 0; i < LT_SENSORS_FRONT; i++)
        if (FrontSensors.Values[i] == true)
        {
            FrontSensors.iMinIndex = i;
            break;    
        }

    // Find the maximum index value that is true
    for (int i = LT_SENSORS_FRONT-1; i >= 0 ; i--)
        if (FrontSensors.Values[i] == true)
        {
            FrontSensors.iMaxIndex = i;
            break;    
        }

    char szNum[10];
    frc::SmartDashboard::PutBoolean("Line Found", FrontSensors.bLineFound);
    for(int i = 0; i < LT_SENSORS_FRONT; i++) 
    {
        sprintf(szNum, "%d", i);
        frc::SmartDashboard::PutBoolean(szNum, FrontSensors.Values[i]);
    }
}


// ----------------------------------------------------------------------------

void ColorSens::UpdateBackValues()
{
    // Read the values and see if a line was found
    BackSensors.bLineFound = false;
    for(int i = 0; i < LT_SENSORS_BACK; i++)
    {
        BackSensors.Values[i] = BackSens[i].Get();
        if (BackSensors.Values[i] == true)
            BackSensors.bLineFound = true;
    }

    // Find the minimum index value that is true
    for (int i = 0; i < LT_SENSORS_BACK; i++)
        if (BackSensors.Values[i] == true)
        {
            BackSensors.iMinIndex = i;
            break;    
        }

    // Find the maximum index value that is true
    for (int i = LT_SENSORS_BACK-1; i >= 0 ; i--)
        if (BackSensors.Values[i] == true)
        {
            BackSensors.iMaxIndex = i;
            break;
        }

    char szNum[10];
    frc::SmartDashboard::PutBoolean("Line Found", BackSensors.bLineFound);
    for(int i = 0; i < LT_SENSORS_BACK; i++) 
    {
        sprintf(szNum, "%d", i);
        frc::SmartDashboard::PutBoolean(szNum, BackSensors.Values[i]);
    }

}


// ----------------------------------------------------------------------------

// Return an error term from -1.0 to 1.0, or InitStrafe if line not found
double ColorSens::GetStrafe(float InitStrafe, bool AllowStrafe)
{
    this->UpdateValues();
    if (FrontSensors.bLineFound && AllowStrafe)
    {
        return (Strafes[FrontSensors.iMinIndex] + Strafes[FrontSensors.iMaxIndex]) / 2.0;
    }
    else
    {
        return InitStrafe;
    }
    
}


// ----------------------------------------------------------------------------

// Return an error term from -1.0 to 1.0, or InitRotate if line not found
double  ColorSens::GetRotate(float InitRotate, bool AllowRotate)
{
    this->UpdateBackValues();
    if (BackSensors.bLineFound && AllowRotate)
    {
        return (Rotates[BackSensors.iMinIndex] + Rotates[BackSensors.iMaxIndex]) / 2.0;
    }
    else 
    {
        return InitRotate;
    }
}

