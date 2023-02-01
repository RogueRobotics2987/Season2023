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
namespace autos {

/**
 * A simple auto that drives forward, then stops.
 */
frc2::CommandPtr SimpleAuto(DriveSubsystem* m_drive);

/**
 * A complex auto command that drives forward, releases a hatch, and then drives
 * backward.
 */
// frc2::CommandPtr ComplexAuto(DriveSubsystem* drive, HatchSubsystem* hatch);

}  // namespace autos
