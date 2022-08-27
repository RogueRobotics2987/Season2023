// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() {

    SetName("ShooterSubsystem");

}

void ShooterSubsystem::Shoot(double speed){
    Shooter->Set(speed);
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {}
