// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class InPlaceRotationCmd
    : public frc2::CommandHelper<frc2::CommandBase, InPlaceRotationCmd> {
 public:
  InPlaceRotationCmd(double angle, DriveSubsystem &drive);

  double DistanceBetweenAngles(double angle1, double angle2);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  double m_angle;

 private:
  DriveSubsystem* m_drive;
  units::angular_velocity::radians_per_second_t max_rot_speed = 4.17_rad_per_s; //4.17
  units::angular_velocity::radians_per_second_t mid_rot_speed = 0.785_rad_per_s; //1.57
  double outer_band = 30;
  double inner_band = 10;
  double m_dist;
  double turn_amount;
  double rot_kp = 0.045;
  double constant_add = 0.3;

};
