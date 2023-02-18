// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Compressor.h"

CompressorObject::CompressorObject() {

}

// This method will be called once per scheduler run
void CompressorObject::Periodic() {
    if (debugCompressorEnabled){
        bool pressureSwitch = phCompressor.GetPressureSwitchValue();
        //double current = phCompressor.GetCompressorCurrent(); //was GetComressorCurrent but that doesn't exist in c++  
        isEnabled = phCompressor.Enabled(); //checks if the compressor is enabled

        frc::SmartDashboard::PutBoolean("Compressor enabled", isEnabled);
        frc::SmartDashboard::PutBoolean("Compressor pressureSwitch", pressureSwitch);
        //frc::SmartDashboard::PutNumber("Compressor current", current);
    }
}



void CompressorObject::StartCompressor() {
    phCompressor.EnableDigital();
}

void CompressorObject::DisableCompressor(){
    phCompressor.Disable();
}

