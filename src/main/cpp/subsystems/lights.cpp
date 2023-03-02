// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/lights.h"

Lights::Lights() = default;

// This method will be called once per scheduler run
// false!
void Lights::Periodic() {
    if (cubeDesired) {
        cubeoutput.Set(1);
        coneoutput.Set(0);
        redoutput.Set(0);
        blueoutput.Set(0);
    } else if (coneDesired) {
        cubeoutput.Set(0);
        coneoutput.Set(1);
        redoutput.Set(0);
        blueoutput.Set(0);
    } else if (redColor) {
        cubeoutput.Set(0);
        coneoutput.Set(0);
        redoutput.Set(1);
        blueoutput.Set(0);
    } else if (blueColor) {
        cubeoutput.Set(0);
        coneoutput.Set(0);
        redoutput.Set(0);
        blueoutput.Set(1);
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

frc2::CommandPtr Lights::AllianceColorCmdPtr(){
    return this->RunOnce(
        [this] {
        if(AllianceColor == frc::DriverStation::Alliance::kRed){
            cubeDesired = false; 
            coneDesired = false;
            redColor = true;
            blueColor = false;  
        }
        else if(AllianceColor == frc::DriverStation::Alliance::kBlue){
            cubeDesired = false; 
            coneDesired = false;
            redColor = false;
            blueColor = true;
        }
        else{
            cubeDesired = false; 
            coneDesired = false;
            redColor = true;
            blueColor = false;
        }
    });
}