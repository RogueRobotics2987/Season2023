// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoAlign.h"

AutoAlign::AutoAlign(){}
AutoAlign::AutoAlign(DriveSubsystem& l_drive) {
  m_drive = &l_drive;
  AddRequirements({m_drive});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutoAlign::Initialize() {
   kp = -0.01f;
}

// Called repeatedly when this Command is scheduled to run
void AutoAlign::Execute() {
  float error = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
  float adjustment = error * kp;
  m_drive->Drive(0_mps, 0_mps, 1_rad_per_s * adjustment, false, false);
}

// Called once the command ends or is interrupted.
void AutoAlign::End(bool interrupted) {
    m_drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, false);
}

// Returns true when the command should end.
bool AutoAlign::IsFinished() {
  return false;
}
