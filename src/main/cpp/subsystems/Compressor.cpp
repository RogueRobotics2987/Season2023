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
        double current = phCompressor.GetCompressorCurrent();
        frc::SmartDashboard::PutBoolean("Compressor enabled", isEnabled);
        frc::SmartDashboard::PutBoolean("pressureSwitch", pressureSwitch);
        frc::SmartDashboard::PutNumber("Compressor current", current);
    }
}



void CompressorObject::StartCompressor() {
    //m_compressor->Start();
   // phCompressor.SetClosedLoopControl(true); //Don't think this is need because ClosedLoopControl is the default

    phCompressor.EnableDigital();
    isEnabled = phCompressor.Enabled(); //checks if the com[ressor is enabled
}

void CompressorObject::DisableCompressor(){
    phCompressor.Disable();
}

