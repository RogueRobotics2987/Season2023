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

    if(pov == 0){
        cubeDesired = true;
        redColor = false;
        blueColor = false;
        coneDesired = false;
    }
    else if(pov == 90){
        cubeDesired = true;
        redColor = true;
        blueColor = false;
        coneDesired = false;
    }
    else if(pov == 180){
        cubeDesired = true;
        redColor = false;
        blueColor = true;
        coneDesired = false;
    }
    else if(pov == 270){
        cubeDesired = true;
        redColor = false;
        blueColor = false;
        coneDesired = true;
    }
    else if(pov == -1){
        std::cout <<"DPAD not being pressed" << std::endl;
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

void Lights::SetLights(int POV){
    pov = POV;
    // std::cout << "DPAD POV " + pov << std::endl;
}