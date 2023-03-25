// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TimedBalanceCmd
    : public frc2::CommandHelper<frc2::CommandBase, TimedBalanceCmd> {
 public:
  TimedBalanceCmd(std::string direction, DriveSubsystem &m_drive);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DriveSubsystem* m_drive;
  int m_state;
  std::string m_direction;
  frc::Timer m_timer;
  units::velocity::meters_per_second_t speed = 1_mps;
  units::velocity::meters_per_second_t actSpeed;

  // frc2::ParallelRaceGroup* AutoBal = new ParallelRaceGroup(TimerCMD(3), m_drive.Drive());
};
