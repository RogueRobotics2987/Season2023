// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Servo.h>
#include <frc2/command/CommandPtr.h>

class lights : public frc2::SubsystemBase {
 public:
  lights();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  frc2::CommandPtr CubeDesired(bool input);
  frc2::CommandPtr ConeDesired(bool input);
 private:
 frc::Servo exampleServo {1};
 bool cubedesired = true;
 bool conedesired = false;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
