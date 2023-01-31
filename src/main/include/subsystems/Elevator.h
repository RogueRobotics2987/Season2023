// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include <frc2/command/CommandPtr.h>

#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>


class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();

  frc2::CommandPtr MotorMoveCommand();
  frc2::CommandPtr StopMoveCommand();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Elevator::ElevatorVert(double elevatorUp, double elevatorDown);
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_vertElevatorMotorLeft = rev::CANSparkMax(59, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax m_vertElevatorMotorRight = rev::CANSparkMax(60, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax m_tiltElevatorMotor = rev::CANSparkMax(61, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax m_clawMotor = rev::CANSparkMax(62, rev::CANSparkMax::MotorType::kBrushless);
  //claw open and close on pneumatics
  double verticalVal = 0.0;
  bool enableElevatorVert = true;
  double upDeadzone; //was 0.08 on the climber for Jaws
  double downDeadzone;
};
