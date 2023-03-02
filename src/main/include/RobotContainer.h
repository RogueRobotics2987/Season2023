// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h> //TODO this libaray list is bloated
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/interfaces/Gyro.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <cameraserver/CameraServer.h>
#include <iostream>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <utility>

#include "Constants.h"
#include "commands/BeginCompressor.h"
#include "commands/ElevatorCmd.h"
#include "commands/AutoBalance.h"
#include "subsystems/Lights.h"
#include "commands/AutoBalance.h"
#include "subsystems/SwerveModule.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Elevator.h"
#include "commands/PlaceAutoCmd.h"

using namespace DriveConstants;
using namespace pathplanner;
using namespace frc2;



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


  // void ConfigureBindings();
  frc2::Command* GetAutonomousCommand();

  frc2::CommandPtr DriveCrgStnBlue1(DriveSubsystem &m_drive);
  frc2::CommandPtr DriveCrgStnBlue2(DriveSubsystem &m_drive);

  frc2::CommandPtr DriveCrgStnRed1(DriveSubsystem &m_drive);
  frc2::CommandPtr DriveCrgStnRed2(DriveSubsystem &m_drive);

  frc2::CommandPtr PlaceDriveCrgStnRed1(DriveSubsystem &m_drive);
  frc2::CommandPtr PlaceDriveCrgStnBlue1(DriveSubsystem &m_drive);

  frc2::CommandPtr ConeBalanceBlue(DriveSubsystem &m_drive);
  frc2::CommandPtr ConeBalanceRed(DriveSubsystem &m_drive);

  


  double GetHeading();

  double GetOdometry();
  
  void ZeroHeading();

  void ConfigMotorControllers();

  void ResetOdometry();

  float Deadzone(float x);

  frc2::CommandPtr GetPathCommand();

 private:
  // The driver's controller
  //frc::Joystick m_stick1{2};
  frc::XboxController m_xbox{0};
  frc::XboxController m_newXbox{1};
  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  Elevator m_elevator;
  CompressorObject m_compressor;
  Lights m_lights;

  //frc2::Command *AutoCmd;

  frc2::Command* AutoCmd = new AutoBalance(m_drive, m_newXbox);

  frc2::Command* PlaceHighCmd = new PlaceAutoCmd(m_elevator, 104, -60, 260); // 104 for height
  frc2::Command* PickupCmd = new PlaceAutoCmd(m_elevator, 70.4, -90, 108);
  frc2::Command* RetractCmd = new PlaceAutoCmd(m_elevator, 5, 0, 0);

  // The chooser for the autonomous routines 
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();

  frc2::CommandPtr DriveCrgStnRed1cmd = DriveCrgStnRed1(m_drive);
  frc2::CommandPtr DriveCrgStnRed2cmd = DriveCrgStnRed2(m_drive);

  frc2::CommandPtr DriveCrgStnBlue1cmd = DriveCrgStnBlue1(m_drive);
  frc2::CommandPtr DriveCrgStnBlue2cmd = DriveCrgStnBlue2(m_drive);

  frc2::CommandPtr PlaceDriveCrgStnRed1cmd = PlaceDriveCrgStnRed1(m_drive);
  frc2::CommandPtr PlaceDriveCrgStnBlue1cmd = PlaceDriveCrgStnBlue1(m_drive); 

  frc2::CommandPtr ConeBalanceBlueCmd = ConeBalanceBlue(m_drive); 
  frc2::CommandPtr ConeBalanceRedCmd = ConeBalanceRed(m_drive);  


  frc2::CommandPtr AutoZeroHeading = m_drive.ZeroHeading();

  frc2::CommandPtr OpenClawCmd = m_elevator.ClawOpenCommand();
  frc2::CommandPtr CloseClawCmd = m_elevator.ClawCloseCommand();

  frc2::CommandPtr SetHighCmd = m_elevator.SetPlaceHighState();
  frc2::CommandPtr SetMidCmd = m_elevator.SetPlaceMidState();
  frc2::CommandPtr SetLowCmd = m_elevator.SetPlaceLowState();





};
