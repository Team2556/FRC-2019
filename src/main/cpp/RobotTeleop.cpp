/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "RobotTeleop.h"

// Constructor
RobotTeleop::RobotTeleop(Robot * pRobot) 
    {
    this->pRobot = pRobot;
    }


// ----------------------------------------------------------------------------

void RobotTeleop::Init()
    {
    }


// ----------------------------------------------------------------------------

void RobotTeleop::Periodic()
    {
    pRobot->RobotDrive.DriveCartesian(
        pRobot->Xbox1.GetX(frc::GenericHID::kRightHand), 
        pRobot->Xbox1.GetY(frc::GenericHID::kRightHand), 
        0);
    }
