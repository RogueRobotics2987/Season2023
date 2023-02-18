// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/lights.h"

Lights::Lights() = default;

// This method will be called once per scheduler run
void Lights::Periodic() {
    if (cubeDesired) {
        output1.Set(1);
        output2.Set(0);
        output3.Set(0);
        output4.Set(0);
    } else if (coneDesired) {
        output1.Set(0);
        output2.Set(1);
        output3.Set(0);
        output4.Set(0);
    } else if (redColor) {
        output1.Set(0);
        output2.Set(0);
        output3.Set(1);
        output4.Set(0);
    } else if (blueColor) {
        output1.Set(0);
        output2.Set(0);
        output3.Set(0);
        output4.Set(1);
    }
}

frc2::CommandPtr Lights::CubeDesired() {
    return this->RunOnce(
        [this] {
            cubeDesired = true; 
            coneDesired = false;
            redColor = false;
            blueColor = false; });
}

frc2::CommandPtr Lights::ConeDesired() {
    return this->RunOnce(
        [this] {
            cubeDesired = false; 
            coneDesired = true;
            redColor = false;
            blueColor = false; });
}

frc2::CommandPtr Lights::RedColor() {
    return this->RunOnce(
        [this] {
            cubeDesired = false; 
            coneDesired = false;
            redColor = true;
            blueColor = false; });
}
frc2::CommandPtr Lights::BlueColor() {
    return this->RunOnce(
        [this] {
            cubeDesired = false; 
            coneDesired = false;
            redColor = false;
            blueColor = true; });
}