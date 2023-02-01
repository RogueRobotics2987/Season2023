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

using namespace pathplanner;

frc2::CommandPtr autos::SimpleAuto(DriveSubsystem* m_drive) {
    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    PathPlannerTrajectory examplePath = PathPlanner::loadPath("New Path", PathConstraints(1.5_mps, 0.5_mps_sq));
    //TODO change Path constraints to Constants

   // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
  // for every path in the group
  // std::vector<PathPlannerTrajectory> pathGroup = PathPlanner::loadPathGroup("fullAuto", {PathConstraints(4_mps, 3_mps_sq)});

  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands/autobuilders.
  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;
  // eventMap.emplace("marker1", std::make_shared<frc2::PrintCommand>("Passed Marker 1"));
  // eventMap.emplace("intakeDown", std::make_shared<IntakeDown>());

  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this could be in RobotContainer along with your subsystems

  SwerveAutoBuilder autoBuilder(
      [m_drive]() { return m_drive->GetPose(); }, // Function to supply current robot pose
      [m_drive](auto initPose) { m_drive->ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(ModuleConstants::kPModuleDriveController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(ModuleConstants::kPModuleTurningController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [m_drive](frc::ChassisSpeeds speeds) { m_drive->Drive(speeds.vx, speeds.vy, speeds.omega, true); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      { &m_drive }, // Drive requirements, usually just a single drive subsystem
      false // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );
  

  //  frc2::SequentialCommandGroup* SnakeSwerve = new frc2::SequentialCommandGroup(

  //   // std::move((autoBuilder.fullAuto(examplePath)))

  //  std::move(autoBuilder.fullAuto(examplePath)) //examplePathCmdPtr

    
  //  );
 
  // return SnakeSwerve;

  return autoBuilder.fullAuto(examplePath); //examplePathCmdPtr
  // Command* fullAutoRawPointer = examplePathCmdPtr.get();
  // return fullAutoRawPointer.get();


  return frc2::SequentialCommandGroup(
             // Reset encoders on command start
              frc2::InstantCommand([] {frc::SmartDashboard::PutNumber("Shooter Set RPM 2 F", 3050);  
                 frc::SmartDashboard::PutNumber("Shooter Set RPM 2 B", 3050);})
            //  // Drive forward while the command is executing
            // //  [drive] { drive->ArcadeDrive(AutoConstants::kAutoDriveSpeed, 0); },
            //  // Stop driving at the end of the command
            //  [drive](bool interrupted) { drive->ArcadeDrive(0, 0); },
            //  // End the command when the robot's driven distance exceeds the
            //  // desired value
            // //  [drive] {
            // //    return drive->GetAverageEncoderDistance() >=
            // //           // AutoConstants::kAutoDriveDistanceInches;
            // //  },
            //  // Requires the drive subsystem
            //  {drive}
            )
      .ToPtr(); //returns command pointers
};
