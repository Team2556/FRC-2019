/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "frc/WPILib.h"
#include "Robot.h"
#include "Elevator.h"

Elevator::Elevator(Robot * pRobot) 
{
    this->pRobot = pRobot;

    //create new DoubleSolenoid//
    hatchSolenoid = new frc::DoubleSolenoid(11,0,1);
}

void Elevator::Output()
{
    //Pneumatic Controlls
    if(pRobot->Xbox2.GetYButtonPressed())
    {
        hatchSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    }
    else if(pRobot->Xbox2.GetYButtonReleased())
    {
        hatchSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);      
    }
}
