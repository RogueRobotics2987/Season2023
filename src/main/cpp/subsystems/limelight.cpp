// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/limelight.h"

limelight::limelight() = default;

// This method will be called once per scheduler run
void limelight::Periodic() {}
    
    frc2::CommandPtr limelight::startCamera(){
        //photonlib::PhotonPipelineResult result = camera.GetLatestResult();
        //nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber

        //nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipline", cur_pipeline);
    }
