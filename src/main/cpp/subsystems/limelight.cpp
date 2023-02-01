// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/limelight.h"
#include <iostream>
#include <span>
using namespace std;

limelight::limelight() = default;

// This method will be called once per scheduler run
void limelight::Periodic() {

}
    
frc2::CommandPtr limelight::ConfigOdometry(){
    return this ->RunOnce( [this] {
        //numAT = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("apriltagsvisable", 0);

        /*if(numAT > 1){
            cout << numAT << endl;
        } else {
            cout << numAT << endl;
        }*/
        double default_array[6] = {};

        std::vector<double> botpose = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose", std::span{default_array, std::size(default_array)});
        // double position[1] = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetDoubleArrayTopic("botpose");
        // double position[] = {nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetDoubleArrayTopic()};

        for(int i=0; i<6; i++){
            cout << botpose[i] << ", ";
        }

        //nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
        //frc::SmartDashboard::PutNumber("testing", tx);
    });
}
