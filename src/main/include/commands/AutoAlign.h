// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/geometry/Pose2d.h>

#include "subsystems/lights.h"
#include "subsystems/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoAlign
    : public frc2::CommandHelper<frc2::CommandBase, AutoAlign> {
 public:
  AutoAlign();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;


 private:
  Lights m_light; 
  DriveSubsystem m_drive;

  std::string sideOfField = "red"; // red or blue
  int grid = 2; // 1 is right, 2 middle, 3 left of robot
  std::string column; // left, middle, right 

  double destinationX;
  double destinationY;

  frc::Pose2d curPosition;
};
