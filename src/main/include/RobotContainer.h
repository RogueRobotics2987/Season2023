// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <subsystems/lights.h>
#include <commands/LightsCmd.h>
#include "Constants.h"
#include <frc/XboxController.h>
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();


 private:
  frc::XboxController m_xbox{0};
  frc::XboxController m_newXbox{1};
  // The robot's subsystems are defined here...
  lights m_lights;

  void ConfigureBindings();
};
