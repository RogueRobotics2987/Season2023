// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include "subsystems/Elevator.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ElevatorCmd
    : public frc2::CommandHelper<frc2::CommandBase, ElevatorCmd> {
 public:
  ElevatorCmd(Elevator& elevator, frc::Joystick& xbox, frc::Joystick& stick1);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Elevator* m_elevator = nullptr;
  frc::Joystick* m_xbox = nullptr;
  frc::Joystick* m_stick1 = nullptr;
};
