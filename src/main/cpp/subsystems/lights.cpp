// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/lights.h"

lights::lights() = default;

// This method will be called once per scheduler run
void lights::Periodic() {
    if(cur_stickPOV != -1){ // if POV has been pressed
        // turn desired LED pattern to true, rest to false
        if(cur_stickPOV == 0){
            cubedesired = true;

            conedesired = false;
            redcolor = false;
            bluecolor = false;

        } else if(cur_stickPOV == 90){
            conedesired = true;

            cubedesired = false;
            redcolor = false;
            bluecolor = false;

        } else if(cur_stickPOV == 180){
            redcolor = true;

            cubedesired = false;
            conedesired = false;
            bluecolor = false;

        } else if(cur_stickPOV == 270){
            bluecolor = true;

            cubedesired = false;
            conedesired = false;
            redcolor = false;
        }
    }

    // start desired LED pattern if one is true
    if (cubedesired) {
        output4.Set(0);
        output2.Set(0);
        output1.Set(1);
        output3.Set(0);
        //exampleServo.SetAngle(0);
        // set to send or not send electrical signal to notify arduino to turn purple.
    }
    else if (conedesired) {
        output4.Set(0);
        output2.Set(1);
        output1.Set(0);
        output3.Set(0);
        //exampleServo.SetAngle(360);
        // set to send or not send electrical signal to notify arduino to turn yellow.
    }
    else if (redcolor) {
        output4.Set(0);
        output2.Set(0);
        output1.Set(0);
        output3.Set(1);
        //you get the point.
    } 
    else if (bluecolor) {
        output4.Set(1);
        output2.Set(0);
        output1.Set(0);
        output3.Set(0);
        // why are you still looking at these
    } 
}
void lights::setStickPOV(int stickPOV){
    if(stickPOV != -1){
        cur_stickPOV = stickPOV;
    }
}

frc2::CommandPtr lights::CubeDesired() {
        return this->RunOnce(
    [this] { 
        cubedesired = true;
        conedesired = false;
        redcolor = false;
        bluecolor = false;
     });
}
frc2::CommandPtr lights::ConeDesired() {
        return this->RunOnce(
    [this] {
        conedesired = true;
        cubedesired = false;
        redcolor = false;
        bluecolor = false;
     });
}
frc2::CommandPtr lights::RedColor() {
        return this->RunOnce(
    [this] {
    redcolor = true;
    conedesired = false;
    cubedesired = false;
    bluecolor = false;
     });
}
frc2::CommandPtr lights::BlueColor() {
        return this->RunOnce(
    [this] {
    bluecolor = true;
    conedesired = false;
    cubedesired = false;
    redcolor = false;
     });
}