// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// #pragma once

// #include <frc2/command/CommandHelper.h>
// #include <frc2/command/SequentialCommandGroup.h>
// #include <frc2/command/Commands.h>
// #include <frc2/command/FunctionalCommand.h>
// #include <frc/smartdashboard/SmartDashboard.h>
// #include <pathplanner/lib/auto/SwerveAutoBuilder.h>
// #include <pathplanner/lib/PathPlanner.h>
// #include <pathplanner/lib/commands/FollowPathWithEvents.h>
// #include <frc/smartdashboard/SendableChooser.h>


// #include "subsystems/DriveSubsystem.h"

// using namespace pathplanner;

// class SequentialAuto
//     : public frc2::CommandHelper<frc2::SequentialCommandGroup,
//                                  SequentialAuto> {
//  public:
//   SequentialAuto(DriveSubsystem m_drive);

//     frc2::CommandPtr DrivePath1(DriveSubsystem &m_drive);

//     frc2::CommandPtr DrivePath2(DriveSubsystem &m_drive);

//   private:
//     frc::SendableChooser<frc2::Command*> c_chooser;

//   DriveSubsystem *c_drive;
  
//     frc2::CommandPtr Drive1 = DrivePath1(*c_drive);
//     frc2::CommandPtr Drive2 = DrivePath2(*c_drive);

// };
