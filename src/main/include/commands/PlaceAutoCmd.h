// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Elevator.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class PlaceAutoCmd
    : public frc2::CommandHelper<frc2::CommandBase, PlaceAutoCmd> {
 public:
  PlaceAutoCmd();
  PlaceAutoCmd(Elevator &elevator, double heightRevolutions, double armAngle, double tiltRevolutions);
  bool IsClose(double check1, double check2);
  


  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  Elevator* m_elevator = nullptr;
  double m_armAngle;
  double m_heightRevolutions;
  double m_tiltRevolutions;
  double m_tiltVelocity;
  


};
