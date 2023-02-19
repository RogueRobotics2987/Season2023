// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoAlign.h"

AutoAlign::AutoAlign() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutoAlign::Initialize() {
    // set sideOfField to red or blue
    /*if(sideOfField == 'red'){
        sideOfField = RED;
    } else if(sideOfField == 'blue'){
        sideOfField = BLUE;
    }*/

    // input user choice for 
    grid = 2;
    column = 'right';

}

// Called repeatedly when this Command is scheduled to run
void AutoAlign::Execute() {}

// Called once the command ends or is interrupted.
void AutoAlign::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoAlign::IsFinished() {
  return false;
}
