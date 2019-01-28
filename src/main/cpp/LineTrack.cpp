/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "LineTrack.h"
#include "Robot.h"

ColorSens::ColorSens() 
{
}

bool ColorSens::LineFound()
{
    for (int i = 0; i < 6; i++)
    {
        if(RLSens[i].Get())
        {
            return true;
        }
    }
    GyroReset = false;
    return false;
}

bool ColorSens::BackLineFound()
{
    if (this->LineFound() == false)
    {
        return false;
    }
    for (int i = 0; i < 2; i++)
    {
        if(BackSens[i].Get())
        {
            return true;
        }
    }
    return false;
}

void ColorSens::UpdateValues()
{
    frc::SmartDashboard::PutBoolean("Line Found", this->LineFound());

    frc::SmartDashboard::PutBoolean("0", this->RLSens[0].Get());
    frc::SmartDashboard::PutBoolean("1", this->RLSens[1].Get());
    frc::SmartDashboard::PutBoolean("2", this->RLSens[2].Get());
    frc::SmartDashboard::PutBoolean("3", this->RLSens[3].Get());
    frc::SmartDashboard::PutBoolean("4", this->RLSens[4].Get());
    frc::SmartDashboard::PutBoolean("5", this->RLSens[5].Get());
    for(int i = 0; i < 6; i++)
    {
        Values[i] = RLSens[i].Get();
    }
}

void ColorSens::UpdateBackValues()
{
    frc::SmartDashboard::PutBoolean("Back Line Found", this->BackLineFound());
    frc::SmartDashboard::PutBoolean("Back Right", this->BackSens[0].Get());
    frc::SmartDashboard::PutBoolean("Back Left", this->BackSens[1].Get());
}

double ColorSens::GetStrafe(float InitStrafe, bool AllowStrafe)
{
    this->UpdateValues();
    if (this->LineFound() && AllowStrafe)
    {
        int Min, Max;
        this->FindMinMax(&Min, &Max);
        return (Strafes[Min]+Strafes[Max])/2;
    }
    else
    {
        return InitStrafe;
    }
    
}

double  ColorSens::GetRotate(float InitRotate, bool AllowRotate)
{
    this->UpdateBackValues();
    if (this->BackLineFound() && AllowRotate)
    {
        int Min, Max;
        this->BackMinMax(&Min, &Max);
        return (Rotates[Min]+Rotates[Max])/2;
    }
    else 
    {
        return InitRotate;
    }
}

void ColorSens::FindMinMax(int * Min, int * Max)
{
    *Min = -1;
    for (int i = 0; i < 6; i++)
    {
        if (Values[i] == true)
        {
            *Max = i;
            if (*Min == -1)
            {
                *Min = i;
            }
        }
    }
}

void ColorSens::BackMinMax(int * Min, int * Max)
{
    *Min = -1;
    for (int i = 0; i < 2; i++)
    {
        if (BackSens[i].Get() == true)
        {
            *Max = i;
            if (*Min == -1)
            {
                *Min = i;
            }
        }
    }
}





void    ColorSens::SetRotate(NavGyro * pNavGyro, bool AllowRotate)
{
    if (this->BackLineFound() && AllowRotate)
    {
        pNavGyro->SetCommandYaw(FindClose(pNavGyro->GetYaw()));
    }
}


float     ColorSens::FindClose(float Angle)
{
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