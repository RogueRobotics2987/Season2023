// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Compressor.h"

CompressorObject::CompressorObject() {

}

// This method will be called once per scheduler run
void CompressorObject::Periodic() {}



void CompressorObject::StartCompressor() {
    //m_compressor->Start();
   // phCompressor.SetClosedLoopControl(true); //Don't think this is need because ClosedLoopControl is the default

    phCompressor.EnableDigital();
    enabled = phCompressor.Enabled();
    frc::SmartDashboard::PutBoolean("Compressor enabled", enabled);
}

void CompressorObject::DisableCompressor(){
    phCompressor.Disable();

}

