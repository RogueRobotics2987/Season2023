// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TimerCMD.h"

TimerCMD::TimerCMD(double l_time) {
  // Use addRequirements() here to declare subsystem dependencies.
    m_time = l_time;

}

// Called when the command is initially scheduled.
void TimerCMD::Initialize() {
  m_timer.Reset();
  m_timer.Start();

}

// Called repeatedly when this Command is scheduled to run
void TimerCMD::Execute() {}

// Called once the command ends or is interrupted.
void TimerCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool TimerCMD::IsFinished() {
  if(double(m_timer.Get()) < m_time) {
    return false;
  }  else {
    return true;
  }
  }
