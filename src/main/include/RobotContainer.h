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


#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/Joystick.h>
#include <iostream>
#include <units/angle.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

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


  void ConfigureBindings();
  frc2::Command* GetAutonomousCommand();

  void ZeroHeading();

  void ConfigMotorControllers();

  void ResetOdometry();

  frc2::Command* GetPathCommand();

 private:
  // The driver's controller
  frc::Joystick m_stick1{2};
  frc::XboxController m_xbox{0};
  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
   Elevator m_elevator;
  CompressorObject m_compressor;


  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
};
