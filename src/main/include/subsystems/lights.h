// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalOutput.h>
#include <frc2/command/SubsystemBase.h>
//#include <frc/Servo.h>
#include <frc2/command/CommandPtr.h>

class lights : public frc2::SubsystemBase {
 public:
  lights();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  frc2::CommandPtr CubeDesired();
  frc2::CommandPtr ConeDesired();
  frc2::CommandPtr RedColor();
  frc2::CommandPtr BlueColor();
 private:
 frc::DigitalOutput output1 {1};
 frc::DigitalOutput output2 {2};
 frc::DigitalOutput output3 {3};
 frc::DigitalOutput output4 {4};
 //frc::Servo exampleServo {1};
 bool cubedesired = false;
 bool conedesired = false;
 bool redcolor = false;
 bool bluecolor = false;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};