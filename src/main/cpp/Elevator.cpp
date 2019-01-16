/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "Solenoid.h"
#include "Elevator.h"

//define pRobot
Elevator::Elevator(Robot * pRobot)
{
 this->pRobot = pRobot;
}

void Elevator::ElevatorMove() {
  //while LB is pressed, elevator will move up, when released elevator will stop
  if (pRobot->Xbox1.GetBumperPressed(frc::XboxController::kLeftHand))
  {
      pRobot->Elevator_Motor.Set(ControlMode::PercentOutput, 1);
  }
  if (pRobot->Xbox1.GetBumperReleased(frc::XboxController::kLeftHand))
  {
      pRobot->Elevator_Motor.Set(ControlMode::PercentOutput, 0);
  }
  //while RB is pressed, elevator will move down, when released elevator will stop
  if (pRobot->Xbox1.GetBumperPressed(frc::XboxController::kRightHand)){
      pRobot->Elevator_Motor.Set(ControlMode::PercentOutput, -1);
  }
  if (pRobot->Xbox1.GetBumperReleased(frc::XboxController::kRightHand))
  {
      pRobot->Elevator_Motor.Set(ControlMode::PercentOutput, 0);
  }
}
//
void Elevator::input()
{
    if (pRobot->Xbox1.GetAButton()) {
        ACounter++;
        if(ACounter % 2 == 1) {
        pRobot->Intake1.Set(ControlMode::PercentOutput, 1);
        }
        if(ACounter % 2 == 0) {
        pRobot->Intake1.Set(ControlMode::PercentOutput, 0);
        }
    }
}