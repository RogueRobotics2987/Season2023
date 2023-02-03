// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoBalance.h"

AutoBalance::AutoBalance(DriveSubsystem& l_drive) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_drive = &l_drive;
  AddRequirements({m_drive});
}

AutoBalance::AutoBalance(){

}

// Called when the command is initially scheduled.
void AutoBalance::Initialize() {
  m_state = 0;

}

// Called repeatedly when this Command is scheduled to run
void AutoBalance::Execute() {
  
  while(m_state == 0) {
    frc::SmartDashboard::PutNumber("Auto Pitch", m_drive->GetPitch());
      m_drive->Drive(0.4_mps, 0_mps, 0_rad_per_s, false, false);
      if(m_drive->GetPitch() >= 15 || m_drive->GetPitch() <= -15) {
        m_state = 1;
      }
    }
    while(m_state == 1) {
      m_drive->Drive(0.2_mps, 0_mps, 0_rad_per_s, false, false);
      if(m_drive->GetPitch() <= 1 || m_drive->GetPitch() >= -1) {
        m_state = 2;
      }
    }
    while(m_state == 2) {
      m_drive->Drive(0.0_mps, 0_mps, 0_rad_per_s, false, false);
    }
}

// Called once the command ends or is interrupted.
void AutoBalance::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoBalance::IsFinished() {
  return false;
}
