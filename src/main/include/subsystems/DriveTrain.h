// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>


class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();
  void Drive(double y, double z);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
 rev::CANSparkMax* LeftBack = new rev::CANSparkMax(56, rev::CANSparkMax::MotorType::kBrushless);
 rev::CANSparkMax* LeftFront = new rev::CANSparkMax(49, rev::CANSparkMax::MotorType::kBrushless);
 rev::CANSparkMax* RightBack = new rev::CANSparkMax(50, rev::CANSparkMax::MotorType::kBrushless);
 rev::CANSparkMax* RightFront = new rev::CANSparkMax(46, rev::CANSparkMax::MotorType::kBrushless);
 frc::DifferentialDrive* m_robotDrive = nullptr;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
