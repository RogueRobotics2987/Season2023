// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/limelight.h"

limelight::limelight() = default;

// This method will be called once per scheduler run
void limelight::Periodic() {}
    
frc2::CommandPtr limelight::configOdometry(){
    
    //nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);


}
