// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>


class CompressorObject : public frc2::SubsystemBase {
 public:
  CompressorObject();
  //~CompressorObject();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void StartCompressor();
  void DisableCompressor();

  bool debugCompressorEnabled = true;


 private:

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::Compressor phCompressor{1, frc::PneumaticsModuleType::REVPH};  //wpilub
  bool isEnabled = false;

};
