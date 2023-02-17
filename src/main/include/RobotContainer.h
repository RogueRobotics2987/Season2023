// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <utility>

#include <frc/controller/PIDController.h> //TODO this libaray list is bloated
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc/XboxController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/interfaces/Gyro.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/Joystick.h>
#include <iostream>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "commands/Autos.h"
#include "commands/SequentialAuto.h"
#include "commands/AutoBalance.h"

using namespace DriveConstants;
using namespace pathplanner;
using namespace frc2;

#include "commands/AutoBalance.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  frc2::CommandPtr DrivePath1(DriveSubsystem &m_drive);

  frc2::CommandPtr DrivePath2(DriveSubsystem &m_drive);

  double GetHeading();

  double GetOdometry();
  
  void ZeroHeading();

  void ConfigMotorControllers();

  void ResetOdometry();

  frc2::CommandPtr GetPathCommand();

 private:
  // The driver's controller
  frc::Joystick m_driverController{OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;

  //frc2::Command *AutoCmd;

  frc2::Command* AutoCmd = new AutoBalance(m_drive, m_driverController);

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  frc2::CommandPtr m_simpleAuto = autos::SimpleAuto(m_drive);
  frc2::CommandPtr m_complexAuto = autos::ComplexAuto(m_drive);
  frc2::CommandPtr m_CommandAuto = autos::CommandPath(m_drive);
  void ConfigureButtonBindings();

  frc2::CommandPtr Drive1 = DrivePath1(m_drive);
  frc2::CommandPtr Drive2 = DrivePath2(m_drive);
};
