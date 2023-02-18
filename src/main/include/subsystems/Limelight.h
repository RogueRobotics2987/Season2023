// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc2/command/CommandPtr.h>



class Limelight : public frc2::SubsystemBase {
 public:
  Limelight();

  frc2::CommandPtr ToggleLight();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
 bool LightOn = false;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
