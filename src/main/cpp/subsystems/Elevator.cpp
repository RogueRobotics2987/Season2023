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
   //m_vertElevatorMotorRight.SetInverted(true);
   m_vertElevatorMotorRight.Follow(m_vertElevatorMotorLeft, true); //now you only call the left motor
   m_tiltElevatorMotor.SetInverted(false);
   // commented out to test, 2/17
   //frc::SmartDashboard::PutBoolean("Elevator Reset Elevator Finished", resetElevatorFinished);
   re_arm.SetPositionConversionFactor(ElevatorConstants::kArmAnglePerRotation);
   frc::SmartDashboard::PutNumber("Elevator Arm kp", ElevatorConstants::kPModuleArmController);
}

// This method will be called once per scheduler run
void Elevator::Periodic() {
   // frc::SmartDashboard::PutNumber("Elevator verticalVal", verticalVal);
   // frc::SmartDashboard::PutNumber("Elevator tiltVal", tiltVal);
   frc::SmartDashboard::PutNumber("Elevator armPos", armPos);
   // frc::SmartDashboard::PutNumber("Elevator armXboxVal", 10000);


   //Elevator height
   //frc::SmartDashboard::PutBoolean("ELevator Height limit switch", ls_vertElevator.Get());
   //frc::SmartDashboard::PutNumber("Elevator Height Encoder", re_vertElevator.GetPosition());

   //Elevator tilt
   // frc::SmartDashboard::PutBoolean("Elevator Tilt limit switch", ls_tiltElevator.Get());
   // frc::SmartDashboard::PutNumber("Elevator Tilt Encoder", re_tiltElevator.GetPosition());

   // //Elevator arm
   // frc::SmartDashboard::PutBoolean("Elevator Arm limit switch", ls_arm.Get());
   frc::SmartDashboard::PutNumber("Elevator Arm encoder", re_arm.GetPosition());
   double curkPArm = frc::SmartDashboard::GetNumber("Elevator Arm kp", ElevatorConstants::kPModuleArmController);
   m_armPIDController.SetP(curkPArm);  
   //frc::SmartDashboard::PutNumber("ELevator Arm voltage", m_armMotor.GEt)
   frc::SmartDashboard::PutNumber("Elevator state", ElevatorState);

   // frc::SmartDashboard::PutNumber("Elevator vert motor 10 output", m_vertElevatorMotorLeft.GetAppliedOutput());
   if (ElevatorState == FIND_ZERO){
      //limit switch is were the elevator is closest to the ground 
      m_vertElevatorMotorLeft.Set(-0.1);//go to forward limit switch

      //limit switch is where the elevator is all the way tilted towards the back of the robot
      m_tiltElevatorMotor.Set(-0.2);//go to reverse limit switch

      //limit switch is when the arm is all the way up
      //
      m_armMotor.Set(0.05);//go to forward limit switch

      if (ls_arm.Get() == true){
         re_arm.SetPosition(0);
      }
      
      if((ls_vertElevator.Get() == true) && (ls_tiltElevator.Get() == true)) { 
         re_vertElevator.SetPosition(0);
         re_tiltElevator.SetPosition(0);
         //frc::SmartDashboard::PutBoolean("Elevator Reset Elevator Finished", true); //for debugging
         ElevatorState = MANUAL_MODE; 
      } 

   } else if (ElevatorState == MANUAL_MODE){
      //double armMeasPos = ElevatorConstants::kArmAnglePerRotation * re_arm.GetPosition();
      double armOutput = m_armPIDController.Calculate(re_arm.GetPosition(), armPos);
      frc::SmartDashboard::PutNumber("Elevator armOutput", armOutput);
      //frc::SmartDashboard::PutNumber("Elevator armMeaPos", armMeasPos);
      
      //experimentally tested that a positive motor output of 0.037 made the output hold steady at -90 degrees
      m_armMotor.Set(armOutput); 
      m_tiltElevatorMotor.Set(tiltVal);
      m_vertElevatorMotorLeft.Set(verticalVal);

      if (ls_arm.Get() == true){
         re_arm.SetPosition(0);
      }

      if (ls_tiltElevator.Get() == true){
         re_tiltElevator.SetPosition(0);
      }

      if (ls_vertElevator.Get() == true){
         re_vertElevator.SetPosition(0);
      }

   } else if (ElevatorState == PLACE_HIGH){

   } else if (ElevatorState == PLACE_MID){

   } else if (ElevatorState == PLACE_LOW){

   }
}

void Elevator::ElevatorVert(double elevatorUp, double elevatorDown) { 
   // frc::SmartDashboard::PutNumber("ElevatorUp Value", elevatorUp);
   // frc::SmartDashboard::PutNumber("ElevatorDown Value", elevatorDown);
   // frc::SmartDashboard::PutNumber("Elevator verticalVal", verticalVal);

   verticalVal =elevatorUp - elevatorDown;

   if (fabs(verticalVal) < ElevatorConstants::vertDeadzone){
      verticalVal = 0;
   }
   /*if((fabs(elevatorUp) > ElevatorConstants::upDeadzone) && (elevatorDown < ElevatorConstants::downDeadzone) && (enableElevator == true)){
      verticalVal = -(elevatorUp - ElevatorConstants::upDeadzone);
   } else if((fabs(elevatorDown) > ElevatorConstants::downDeadzone) && (elevatorUp < ElevatorConstants::upDeadzone) && (enableElevator == true)){
      verticalVal = (elevatorDown - ElevatorConstants::downDeadzone);
   } else {
      verticalVal = 0;
   }*/
}

void Elevator::ElevatorTilt(double lean){
   if (enableElevator == true){
      tiltVal = -lean;
   } else if (fabs(tiltVal) < ElevatorConstants::tiltDeadzone){
      tiltVal = 0;
   } else {
      tiltVal = 0;
   }


}

void Elevator::ElevatorArm(double armXboxVal){
   // frc::SmartDashboard::PutNumber("Elevator armPos", armPos);

   // frc::SmartDashboard::PutNumber("Elevator armXboxVal", armXboxVal);

   if ((fabs(armXboxVal) < ElevatorConstants::armDeadzone) && (enableElevator == true)) {
      armPos = armPos; //the arm stays in the same position
   } else if (enableElevator == true){
      armPos = armPos + (armXboxVal * (2));
   } else {
      armPos = 0;
   }

   if (armPos > 10){
      armPos = 0;
   } else if (armPos < -180){
      armPos = -180;
   }
}


frc2::CommandPtr Elevator::ClawCloseCommand() {
   return this->RunOnce(
      [this] { clawSolenoid.Set(frc::DoubleSolenoid::kReverse); });
}

frc2::CommandPtr Elevator::ClawOpenCommand() {
   return this->RunOnce(
      [this] { clawSolenoid.Set(frc::DoubleSolenoid::kForward); });
}

frc2::CommandPtr Elevator::SetPlaceHighState(){
  return this->RunOnce(
      [this] { ElevatorState = PLACE_HIGH; });
}

frc2::CommandPtr Elevator::SetPlaceMidState(){
  return this->RunOnce(
      [this] { ElevatorState = PLACE_MID; });
}
frc2::CommandPtr Elevator::SetPlaceLowState(){
   return this->RunOnce(
      [this] { ElevatorState = PLACE_LOW; });
}

frc2::CommandPtr Elevator::SetManualElevatorState(){
  return this->RunOnce(
      [this] { ElevatorState = MANUAL_MODE; });
}

frc2::CommandPtr Elevator::SetArmPos(double angle){
   return this->RunOnce(
      [this, angle] {armPos = angle; });
}