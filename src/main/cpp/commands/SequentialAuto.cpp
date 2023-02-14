// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SequentialAuto.h"


    frc2::CommandPtr SequentialAuto::DrivePath1(DriveSubsystem &m_drive){
      PathPlannerTrajectory examplePath = PathPlanner::loadPath("Move_1", PathConstraints(3_mps, 1_mps_sq));


      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  SwerveAutoBuilder autoBuilder(
      [&m_drive]() { return m_drive.GetPose(); }, // Function to supply current robot pose
      [&m_drive](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(ModuleConstants::kPModuleDriveController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(ModuleConstants::kPModuleTurningController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [&m_drive](frc::ChassisSpeeds speeds) { m_drive.Drive(speeds.vx, speeds.vy, speeds.omega, true, false); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      { &m_drive }, // Drive requirements, usually just a single drive subsystem
      false // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );

  return autoBuilder.followPath(examplePath); //examplePathCmdPtr
  };

      frc2::CommandPtr SequentialAuto::DrivePath2(DriveSubsystem &m_drive){
      PathPlannerTrajectory examplePath = PathPlanner::loadPath("Move_2", PathConstraints(3_mps, 1_mps_sq));


      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  SwerveAutoBuilder autoBuilder(
      [&m_drive]() { return m_drive.GetPose(); }, // Function to supply current robot pose
      [&m_drive](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(ModuleConstants::kPModuleDriveController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(ModuleConstants::kPModuleTurningController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [&m_drive](frc::ChassisSpeeds speeds) { m_drive.Drive(speeds.vx, speeds.vy, speeds.omega, true, false); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      { &m_drive }, // Drive requirements, usually just a single drive subsystem
      false // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );

  return autoBuilder.followPath(examplePath); //examplePathCmdPtr
  };

SequentialAuto::SequentialAuto(DriveSubsystem m_drive)
{
  std::vector<std::unique_ptr<Command>> commands;

  commands.emplace_back(Drive1.get());
  commands.emplace_back(Drive2.get());

  auto group = SequentialCommandGroup(std::move(commands));
  group.Execute();

  
  
}

