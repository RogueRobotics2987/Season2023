// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() = default;

// This method will be called once per scheduler run
void Intake::Periodic() {}

   frc2::CommandPtr Intake::MotorMoveCommand() {
      return this->RunOnce(
      [this] { m_intakeMotor.Set(0.5); });
   }

   frc2::CommandPtr Intake::StopMoveCommand(){
      return this->RunOnce(
         [this] {m_intakeMotor.Set(0); });
   }