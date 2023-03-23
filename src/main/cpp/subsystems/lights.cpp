// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/lights.h"

Lights::Lights() {
    frc::SmartDashboard::PutString("Lights_POV", "default");
}

// This method will be called once per scheduler run
void Lights::Periodic() {

    if(cur_xboxPOV == 0){
        // redColor = false;
        // blueColor = false;
        // cubeDesired = true;
        // coneDesired = false;
        // frc::SmartDashboard::PutString("Lights_POV", "purple");
    } else if (cur_xboxPOV == 90){
        if (allianceColorRed == true){
            redColor = true;
            blueColor = false;
            cubeDesired = false;
            coneDesired = false;
            frc::SmartDashboard::PutString("Lights_POV", "red");

        } else if (allianceColorRed == false){
            redColor = false;
            blueColor = true;
            cubeDesired = false;
            coneDesired = false;
            frc::SmartDashboard::PutString("Lights_POV", "blue");
        }
    } else if (cur_xboxPOV == 180){
        // redColor = false;
        // blueColor = false;
        // cubeDesired = false;
        // coneDesired = true;
        // frc::SmartDashboard::PutString("Lights_POV", "yellow");
    } else if (cur_xboxPOV == 270){

    }

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
            allianceColorRed = true;
        }
        else if(AllianceColor == frc::DriverStation::Alliance::kBlue){
            cubeDesired = false; 
            coneDesired = false;
            redColor = false;
            blueColor = true;
            allianceColorRed = false;
        }
        else{
            cubeDesired = false; 
            coneDesired = false;
            redColor = true;
            blueColor = false;
            allianceColorRed = true;
        }
    });
}

void Lights::SetPOV(int xboxPOV){
    cur_xboxPOV = xboxPOV;
}