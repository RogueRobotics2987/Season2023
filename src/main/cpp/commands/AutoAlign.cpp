// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoAlign.h"

#include <iostream>
using namespace std;

AutoAlign::AutoAlign() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutoAlign::Initialize() {
    // set sideOfField to red or blue
    /*if(DriverStation.GetAlliance() == "red"){
        sideOfField = "red";
        m_light.RedColor();

    } else if(sideOfField == "blue"){
        sideOfField = "blue";
        m_light.BlueColor();
    }*/

    // input user choice for grid & column
    grid = 2;
    column = 'right';
}

// Called repeatedly when this Command is scheduled to run
void AutoAlign::Execute() {
    if(grid == 1){
        // set destination to apriltag right of drivers perspective
        destinationX = 5.41; // tx
        destinationY = -1.31; // ty
    } else if(grid == 3){
        // destination left apriltag
        destinationX = 5.41; // tx
        destinationY = -1.31; // ty
    } else { 
        // defalut grid 2
        // destination middle apriltag
        destinationX = 5.41; // tx
        destinationY = -1.31; // ty
    }

    // move robot along x-axis
    curPosition = m_drive.GetPose(); 

    frc::SmartDashboard::PutData(curPosition.Y().value());

    /*if(curret x pose < destinationX + desOffset){
        m_drive.Drive(0_mps, 3_mps, )
    }*/
    // if reached destination, then

    // move robot in front of wanted column (along x-axis), using + || - set value

}

// Called once the command ends or is interrupted.
void AutoAlign::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoAlign::IsFinished() {
  return false;
}
