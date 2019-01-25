/*
 * NavGyro.h
 *
 *  Created on: Feb 1, 2018
 *      Author: team2556
 */
#define NAVX
//#define ADXRS_GYRO

#ifndef SRC_NAVGYRO_H_
#define SRC_NAVGYRO_H_

#ifdef NAVX
#include <AHRS.h>
#endif

#ifdef ADXRS_GYRO
#include "frc/wpilib.h"
//#include "frc/ADXRS450_Gyro.h"
#endif

class NavGyro
{
public:
    // Constructor / Destructor
    NavGyro();
    virtual ~NavGyro();

private:
    // Data
#ifdef NAVX
    AHRS *  		pNavX;
#endif
#ifdef ADXRS_GYRO
    frc::ADXRS450_Gyro  * pADXRS;
#endif

public:
    float		fGyroCommandYaw;
    bool		bPresetTurning;

    // Methods
    void   Init();
    void   UpdateValues();
    void   SetCommandYaw(float fAngle);
    void   SetCommandYawToCurrent();
    void   ResetYaw();
    float  GetYaw();
    float  GetYawError();
	
    float  CorrectRotate(float fRotateLess);
    float  GetTilt();
	
    float  GetRotate(float fRotateMax = 0.5);
	
#ifdef NAVX
    float	GetDisplacemetX();
    float	GetDisplacemetY();
    float	GetDisplacemetZ();
#endif
    
};

#endif /* SRC_NAVGYRO_H_ */
