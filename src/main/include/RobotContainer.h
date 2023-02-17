// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include "Constants.h"
#include "commands/ElevatorCmd.h"
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "commands/BeginCompressor.h"
#include "subsystems/Compressor.h"
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <cameraserver/CameraServer.h>



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
 
  // The robot's subsystems are defined here...
  Elevator m_elevator;
  CompressorObject m_compressor;


  frc::XboxController m_xbox{0};
  frc::Joystick m_stick1{2};

  void ConfigureBindings();
};
