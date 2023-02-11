// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"
//state machine to hit all the limit switches
//one command SetManipulartorState
//Inside command with three functions, 
//subsystem should have a function that says to go to a specifc encoder value

Elevator::Elevator() {
   m_vertElevatorMotorLeft.SetInverted(false);
   m_vertElevatorMotorRight.SetInverted(false);
   m_vertElevatorMotorRight.Follow(m_vertElevatorMotorLeft); //now you only call the left motor
   m_tiltElevatorMotor.SetInverted(false);

   //m_compressor = new frc::Compressor;
}

// This method will be called once per scheduler run
void Elevator::Periodic() {
   //frc::SmartDashboard::PutNumber("Elevator verticalVal", verticalVal);
   //Elevator height
   frc::SmartDashboard::PutBoolean("ELevator Height limit switch", ls_vertElevator.Get());
   frc::SmartDashboard::PutNumber("Elevator Height Encoder", re_vertElevator.GetPosition());

   //Elevator tilt
   frc::SmartDashboard::PutBoolean("ELevator Tilt limit switch", ls_tiltElevator.Get());
   frc::SmartDashboard::PutNumber("Elevator Tilt Encoder", re_tiltElevator.GetPosition());
   
   frc::SmartDashboard::PutBoolean("Elevator Reset Elevator Finished", resetElevatorFinished);


   if (ElevatorState == FIND_ZERO){
      m_vertElevatorMotorLeft.Set(-0.2);
      m_tiltElevatorMotor.Set(-0.2);
      m_armMotor.Set(-0.2);

        if((ls_vertElevator.Get() == true) && (ls_tiltElevator.Get() == true) && (ls_arm.Get() == true)) { 
            re_vertElevator.SetPosition(0);
            re_tiltElevator.SetPosition(0);
            re_arm.SetPosition(0);
            m_vertElevatorMotorLeft.Set(0);
            m_tiltElevatorMotor.Set(0);
            m_armMotor.Set(0);
            frc::SmartDashboard::PutBoolean("Elevator Reset Elevator Finished", true); //for debugging
            ElevatorState = MANUAL_MODE; 
        } 

   } else if (ElevatorState == MANUAL_MODE){
      m_vertElevatorMotorLeft.Set(verticalVal);
      m_tiltElevatorMotor.Set(tiltVar);
      m_armMotor.Set(armVar);
   } else if (ElevatorState == PLACE_HIGH){

   } else if (ElevatorState == PLACE_MID){

   } else if (ElevatorState == PLACE_LOW){

   }
}

void Elevator::ElevatorVert(double elevatorUp, double elevatorDown) { 
      frc::SmartDashboard::PutNumber("elevatorUp Value", elevatorUp);
      frc::SmartDashboard::PutNumber("elevatorDown Value", elevatorDown);

      if((fabs(elevatorUp) > ElevatorConstants::upDeadzone) && (elevatorDown < ElevatorConstants::downDeadzone) && (enableElevator == true)){
        verticalVal = elevatorUp - ElevatorConstants::upDeadzone;
      } else if((fabs(elevatorDown) > ElevatorConstants::downDeadzone) && (elevatorUp < ElevatorConstants::upDeadzone) && (enableElevator == true)){
        verticalVal = -(elevatorDown - ElevatorConstants::downDeadzone);
      } else {
        verticalVal = 0;
      }
   }

void Elevator::ElevatorTilt(double lean){
   if (enableElevator == true){
      tiltVar = lean;
   } else {
      tiltVar = 0;
   }
}

void Elevator::ElevatorArm(double armXboxVal){
   if (enableElevator == true) {
      armVar = armXboxVal;
   } else {
      tiltVar = 0;
   }
}


/*void Elevator::Close(int SolenoidNum){ //logic copied from t-shirt cannon
    if(SolenoidNum==1){
        std::cout << "Solenoid 1 is closing" << std::endl;
        ShooterSolenoid1.Set(frc::DoubleSolenoid::kForward);
    } 
}
void Elevator::Open(int SolenoidNum){ 
    if(SolenoidNum==1){
        ShooterSolenoid1.Set(frc::DoubleSolenoid::kReverse);
    } 
}*/


   frc2::CommandPtr Elevator::ClawOpenCommand() {
      return this->RunOnce(
         [this] { clawSolenoid.Set(frc::DoubleSolenoid::kReverse); });
   }

frc2::CommandPtr Elevator::ClawCloseCommand() {
      return this->RunOnce(
         [this] { clawSolenoid.Set(frc::DoubleSolenoid::kForward); });
   }


frc2::CommandPtr Elevator::SetPlaceHighState(){
   ElevatorState = PLACE_HIGH;
}

frc2::CommandPtr Elevator::SetPlaceMidState(){
   ElevatorState = PLACE_MID;
}

frc2::CommandPtr Elevator::SetPlaceLowState(){
   ElevatorState = PLACE_LOW;
}

frc2::CommandPtr Elevator::SetManualElevatorState(){
   ElevatorState = MANUAL_MODE;
}