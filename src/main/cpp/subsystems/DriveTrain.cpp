// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"

DriveTrain::DriveTrain() {

    SetName("DriveTrain");

    m_robotDrive = new frc::DifferentialDrive(*LeftFront, *RightFront);
    LeftBack->Follow(*LeftFront);
    RightBack->Follow(*RightFront);
}

void DriveTrain::Drive(double y, double z) {
    m_robotDrive->ArcadeDrive(y, -z);
}

// This method will be called once per scheduler run
void DriveTrain::Periodic() {}
