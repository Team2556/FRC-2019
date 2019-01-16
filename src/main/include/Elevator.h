/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

class Elevator {
public:
Elevator(Robot * pRobot);

void ElevatorMove();
void Pneumatics();
void input();
void output();
//counts # of times "A" is pressed
int solCounter = 0;
int ACounter = 0;
Robot * pRobot;
};
