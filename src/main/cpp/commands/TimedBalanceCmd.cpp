// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TimedBalanceCmd.h"

TimedBalanceCmd::TimedBalanceCmd(std::string direction, DriveSubsystem &l_drive) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_drive = &l_drive;
  AddRequirements({m_drive});
  m_direction = direction;
}
// Called when the command is initially scheduled.
void TimedBalanceCmd::Initialize() {
  m_state = 0;
  m_timer.Stop();
  m_timer.Reset();
}

// Called repeatedly when this Command is scheduled to run
void TimedBalanceCmd::Execute() {
  

  if(m_direction == "Left"){
    actSpeed = speed * 1;
  }
  else if(m_direction == "Right"){
    actSpeed = speed * -1;
  }

  if(m_state == 0){
    m_drive->Drive(0_mps, actSpeed * 2, 0_rad_per_s, false, false);
  
    if(m_drive->GetPitch() >= 9 || m_drive->GetPitch() <= -9){
      m_state = 1;
      m_timer.Start();
    }

  }
  else if(m_state == 1){
    if(m_timer.Get() <= 3.25_s){
    m_drive->Drive(0_mps, actSpeed * 0.45, 0_rad_per_s, false, false);
    }
    else{
      m_state = 2;
    }
  }
  else if(m_state == 2){
    m_drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, false);
    m_state = 3;
    m_timer.Stop();
  }
}

// Called once the command ends or is interrupted.
void TimedBalanceCmd::End(bool interrupted) {

}

// Returns true when the command should end.
bool TimedBalanceCmd::IsFinished() {
  if(m_state == 3){
  return true;
  }
  else {
    return false;
  }
}
