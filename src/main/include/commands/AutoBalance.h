// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoBalance
    : public frc2::CommandHelper<frc2::CommandBase, AutoBalance> {
 public:
  AutoBalance(DriveSubsystem& l_drive, frc::XboxController &l_Joystick);

  AutoBalance();

  void Initialize() override;

  void Periodic();

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  frc2::CommandPtr runCmd(bool run);

  private:
    DriveSubsystem* m_drive;
    int m_state;
    double m_angle;
    frc::Timer m_timer;
    bool backwardsCheck;
    // bool autoCheck;
    frc::XboxController* m_Joystick;
};
