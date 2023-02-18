// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Limelight.h"

Limelight::Limelight() = default;

frc2::CommandPtr Limelight::ToggleLight(){
return this->RunOnce(
    [this] { if(LightOn == false){ nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3); 
    LightOn = true;
    }
    else{
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1); 
        LightOn = false;
    }
    });
}

// This method will be called once per scheduler run
void Limelight::Periodic() {}
