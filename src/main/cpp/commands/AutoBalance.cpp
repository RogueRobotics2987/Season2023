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
// Called repea  
  m_angle = 9;
  frc::SmartDashboard::PutNumber("m_angle value: ", m_angle);
}

// Called repeatedly when this Command is scheduled to run
void AutoBalance::Execute() {
  
  while(m_state == 0) {
    m_angle = frc::SmartDashboard::GetNumber("m_angle value: ", m_angle);
    frc::SmartDashboard::PutNumber("Auto Pitch", m_drive->GetPitch());
      m_drive->Drive(0.4_mps * 2, 0_mps, 0_rad_per_s, false, false);
      if(m_drive->GetPitch() >= m_angle || m_drive->GetPitch() <= -m_angle) {
        m_state = 1;
      }
    }
    while(m_state == 1){
      m_drive->Drive(0.4_mps, 0_mps, 0_rad_per_s, false, false);
      if(m_drive->GetPitch() <= m_angle / 2 && m_drive->GetPitch() >= -m_angle / 2) {
        m_state = 2;
      }
    }
    while(m_state == 2) {
    frc::SmartDashboard::PutNumber("Auto Pitch", m_drive->GetPitch());
        m_drive->Drive(0.0_mps, 0_mps, 0_rad_per_s, false, true);

      if(m_drive->GetPitch() <= -8) {
        m_state = 3;
      }
    }
      while(m_state == 3) {
        m_drive->Drive(0.4_mps * 2, 0_mps, 0_rad_per_s, false, false);
      if(m_drive->GetPitch() >= -8) {
        m_state = 2;
      }
    }
  float curAngle = m_drive->GetPitch();
  units::meters_per_second_t speed = units::meters_per_second_t(0.125 * curAngle);

if(curAngle >= 1 || curAngle <= -1) {
  m_drive->Drive(-speed, 0_mps, 0_rad_per_s, false, false);
} else {
  m_drive->Drive(0.0_mps, 0_mps, 0_rad_per_s, false, false);
}
}
    //while(m_state == 2) {
      // m_drive->Drive(0.0_mps, 0_mps, 0_rad_per_s, false, false);
    //}
void AutoBalance::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoBalance::IsFinished() {
  return false;
}
