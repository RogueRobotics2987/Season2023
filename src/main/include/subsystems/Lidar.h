// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc/XboxController.h>
#include <frc2/command/CommandPtr.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Lidar : public frc2::SubsystemBase {
 public:
  Lidar();
    bool m_lidarVal = false;
    frc::DigitalInput input{6};
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
