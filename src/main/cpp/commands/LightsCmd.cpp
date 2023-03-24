// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LightsCmd.h"

LightsCmd::LightsCmd(Lights &lights, frc::XboxController &xbox, frc::XboxController &newXbox) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_lights = &lights;
  m_xbox = &xbox;
  m_newXbox = &newXbox;
  AddRequirements({m_lights});

}

// Called when the command is initially scheduled.
void LightsCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void LightsCmd::Execute() {
  m_lights->SetPOV(m_xbox->GetPOV());
  m_lights->FindAllianceColor();

}

// Called once the command ends or is interrupted.
void LightsCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool LightsCmd::IsFinished() {
  return false;
}
