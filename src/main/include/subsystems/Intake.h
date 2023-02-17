// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include <frc2/command/CommandPtr.h>

#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  frc2::CommandPtr MotorMoveCommand();
  frc2::CommandPtr StopMoveCommand();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_intakeMotor = rev::CANSparkMax(60, rev::CANSparkMax::MotorType::kBrushless);

};