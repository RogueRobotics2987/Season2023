// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/commands/FollowPathWithEvents.h>


using namespace pathplanner;

frc2::CommandPtr autos::SimpleAuto(DriveSubsystem &m_drive) {
    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    PathPlannerTrajectory examplePath = PathPlanner::loadPath("Simple Path", PathConstraints(3_mps, 1_mps_sq));
   // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
  // for every path in the group
  // std::vector<PathPlannerTrajectory> pathGroup = PathPlanner::loadPathGroup("fullAuto", {PathConstraints(4_mps, 3_mps_sq)});

  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands/autobuilders.
  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;
  // eventMap.emplace("marker1", std::make_shared<frc2::PrintCommand>("Passed Marker 1"));
  // eventMap.emplace("intakeDown", std::make_shared<IntakeDown>());


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

frc2::CommandPtr autos::ComplexAuto(DriveSubsystem &m_drive) {
    PathPlannerTrajectory ComplexPath = PathPlanner::loadPath("New Path", PathConstraints(3_mps, 1_mps_sq));
 
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

  return autoBuilder.followPath(ComplexPath); //examplePathCmdPtr

};

frc2::CommandPtr autos::CommandPath(DriveSubsystem &m_drive) {
    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // PathPlannerTrajectory examplePath = PathPlanner::loadPath("180 Path", PathConstraints(3_mps, 1_mps_sq));
   // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
  // for every path in the group
  std::vector<PathPlannerTrajectory> pathGroup = PathPlanner::loadPathGroup("180 Path", {PathConstraints(4_mps, 3_mps_sq)});

  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands/autobuilders.
  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;
  eventMap.emplace("marker1", std::make_shared<frc2::PrintCommand>("Passed Marker 1"));
  // eventMap.emplace("intakeDown", std::make_shared<IntakeDown>());


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

  return autoBuilder.fullAuto(pathGroup); 

};

// frc2::SequentialCommandGroup MovePath(){
//   PathPlannerTrajectory Move1 = PathPlanner::loadPath("Move_1", PathConstraints(3_mps, 1_mps_sq));
//   PathPlannerTrajectory Move2 = PathPlanner::loadPath("Move_2", PathConstraints(3_mps, 1_mps_sq));

// PathPlannerTrajectory::PathPlannerState exampleState = examplePath.sample(1.2_s);


// }