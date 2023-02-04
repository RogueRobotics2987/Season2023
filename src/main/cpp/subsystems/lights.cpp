// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/lights.h"

lights::lights() = default;

// This method will be called once per scheduler run
void lights::Periodic() {
    if (cubedesired) {
        exampleServo.SetAngle(0);
    }
    else if (conedesired) {
        exampleServo.SetAngle(360);
    }
}

frc2::CommandPtr lights::CubeDesired(bool input) {
    return this->RunOnce(
    [this, input] {
        cubedesired = input;
    conedesired = !input;
     });
}
frc2::CommandPtr lights::ConeDesired(bool input) {
        return this->RunOnce(
    [this, input] {
        conedesired = input;
    cubedesired = !input;
     });
}