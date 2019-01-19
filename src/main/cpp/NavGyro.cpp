/*
 * NavGyro.cpp
 *
 *  Created on: Feb 1, 2018
 *      Author: team2556
 */


#include "frc/WPILib.h"

//#include "RobotMap.h"

#ifdef NAVX
#include <frc/AHRS.h>
#endif

#ifdef ADXRS_GYRO
#include <ADXRS450_Gyro.h>
#endif

#include <NavGyro.h>

float fNormalizeAngle360(float fAngle);
float fNormalizeAngle180(float fAngle);

// ============================================================================
// NavGyro
// ============================================================================

// ----------------------------------------------------------------------------
// Constructor / Destructor
// ----------------------------------------------------------------------------

NavGyro::NavGyro()
    {
#ifdef NAVX
    // Make the NavX control object
    pNavX = new AHRS(SPI::Port::kMXP);
#endif

#ifdef ADXRS_GYRO
    pADXRS = new frc::ADXRS450_Gyro(frc::SPI::Port::kOnboardCS0);
#endif

    }


// ----------------------------------------------------------------------------

NavGyro::~NavGyro()
    {

    }


// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------

void NavGyro::Init()
    {
    // Get the initial starting angle
#ifdef NAVX
    fGyroCommandYaw = pNavX->GetYaw();
    pNavX->ResetDisplacement();
#endif
#ifdef ADXRS_GYRO
//    pADXRS->Calibrate();
    fGyroCommandYaw = pADXRS->GetAngle();
#endif
    }


// ----------------------------------------------------------------------------

#ifdef NAVX
void NavGyro::UpdateValues()
{
	float fAccelX    = pNavX->GetRawAccelX();
	float fAccelY    = pNavX->GetRawAccelY();
	int   UpdateRate = pNavX->GetRequestedUpdateRate();
	bool  bMoving    = pNavX->IsMoving();

	SmartDashboard::PutNumber("Is Moving", bMoving);
	pNavX->UpdateDisplacement(fAccelX,fAccelY,UpdateRate,true);
}
#endif

//-----------------------------------------------------------------------------
void NavGyro::SetCommandYaw(float fAngle)
    {
    fGyroCommandYaw = fNormalizeAngle360(fAngle);
    }


// ----------------------------------------------------------------------------

void NavGyro::SetCommandYawToCurrent()
    {
#ifdef NAVX
    fGyroCommandYaw = pNavX->GetYaw();
#endif
#ifdef ADXRS_GYRO
    fGyroCommandYaw = pADXRS->GetAngle();
#endif
    }


// ----------------------------------------------------------------------------

float NavGyro::GetYaw()
    {
#if defined(NAVX)
    return pNavX->GetYaw(); 
#elif defined(ADXRS_GYRO)
    return pADXRS->GetAngle();
#else
    return 0.0;
#endif
    }


// ----------------------------------------------------------------------------

void NavGyro::ResetYaw()
{
#if defined(NAVX)
    pNavX->Reset();
#endif
    SetCommandYawToCurrent();
}


// ----------------------------------------------------------------------------

float  NavGyro::GetYawError()
    {
    return -fNormalizeAngle180(GetYaw() - fGyroCommandYaw);
    }


//-----------------------------------------------------------------------------

float  NavGyro::CorrectRotate(float fRotateLess)
{
	if(fRotateLess >  0.5)
	{
		fRotateLess =  0.5;
	}
	if(fRotateLess < -0.5)
	{
		fRotateLess = -0.5;
	}
	return fRotateLess;
}


//-----------------------------------------------------------------------------

float  NavGyro::GetRotate()
	{
    float YawError;

    YawError = this->GetYawError() * 0.05;
    YawError = this->CorrectRotate(YawError);
    return YawError;
	}


#if defined(NAVX)
//-----------------------------------------------------------------------------

float	NavGyro::GetDisplacemetX()
{
    // Get X displacement and convert to feet
	return	pNavX->GetDisplacementX() * 3.28084;
}


//-----------------------------------------------------------------------------

float	NavGyro::GetDisplacemetY()
{
    // Get X displacement and convert to feet
	return	pNavX->GetDisplacementY() * 3.28084;
}


//-----------------------------------------------------------------------------

float	NavGyro::GetDisplacemetZ()
    {
    // Get Z displacement and convert to feet
	return	pNavX->GetDisplacementZ()*3.28084;
    }
#endif

// ----------------------------------------------------------------------------
// Utilities
// ----------------------------------------------------------------------------

// Normalize fAngle range from 0.0 to 360.0

float fNormalizeAngle360(float fAngle)
    {
    while (fAngle <    0.0) fAngle += 360.0;
    while (fAngle >= 360.0) fAngle -= 360.0;
    return fAngle;
    }


// ----------------------------------------------------------------------------

// Normalize fAngle range from +180.0 to -180.0

float fNormalizeAngle180(float fAngle)
    {
    while (fAngle <  -180.0) fAngle += 360.0;
    while (fAngle >=  180.0) fAngle -= 360.0;
    return fAngle;
    }





