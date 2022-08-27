// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shooter.h"

Shooter::Shooter() {
  // Use addRequirements() here to declare subsystem dependencies.
  SetName("Shooter");
  AddRequirements({m_shooter});
}

// Called when the command is initially scheduled.
void Shooter::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Shooter::Execute() {
  m_shooter->Shoot(0.5);
}

// Called once the command ends or is interrupted.
void Shooter::End(bool interrupted) {
  m_shooter->Shoot(0.0);
}

// Returns true when the command should end.
bool Shooter::IsFinished() {
  return false;
}
