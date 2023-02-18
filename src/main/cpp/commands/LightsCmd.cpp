// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LightsCmd.h"

LightsCmd::LightsCmd(lights &leds, frc::XboxController &xbox, frc::XboxController &newXbox) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_leds = &leds;
  m_xbox = &xbox;
  m_newXbox = &newXbox;
}

// Called when the command isj initially scheduled.
void LightsCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void LightsCmd::Execute() {
  //m_xbox->GetPOV();
  m_leds->setStickPOV(m_xbox->GetPOV());

}

// Called once the command ends or is interrupted.
void LightsCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool LightsCmd::IsFinished() {
  return false;
}
