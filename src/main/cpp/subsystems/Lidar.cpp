// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lidar.h"

Lidar::Lidar() = default;

// This method will be called once per scheduler run
void Lidar::Periodic() {
    m_lidarVal = input.Get();
    if(m_lidarVal == true) {
        frc::SmartDashboard::PutString("LiDAR Value", "Close the Claw!");
    }
    else {
        frc::SmartDashboard::PutString("LiDAR Value", "Don't Close the Claw!");
    }
}
