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
   m_vertElevatorMotorRight.Follow(m_vertElevatorMotorLeft, true); //now you only call the left motor
   m_tiltElevatorMotor.SetInverted(false);
   //frc::SmartDashboard::PutBoolean("Elevator Reset Elevator Finished", resetElevatorFinished);
   re_arm.SetPositionConversionFactor(ElevatorConstants::kArmAnglePerRotation);
   frc::SmartDashboard::PutNumber("Elevator Arm kp", ElevatorConstants::kPModuleArmController);
   frc::SmartDashboard::PutNumber("Elevator arm max change", armMaxChange);
   frc::SmartDashboard::PutNumber("Elevator vert max change", vertMaxChange);   
   frc::SmartDashboard::PutNumber("Elevator Vert kp", ElevatorConstants::kPModuleVertController);
   //frc::SmartDashboard::PutBoolean("Elevator safe Up and Down", safeForVertElevator);
}

// This method will be called once per scheduler run
void Elevator::Periodic() {

   // frc::SmartDashboard::PutData(frc2::CommandScheduler::GetInstance());
   frc::SmartDashboard::PutNumber("Elevator verticalVal", verticalVal);
   frc::SmartDashboard::PutNumber("Elevator vertOutput", vertOutput);
   frc::SmartDashboard::PutNumber("Elevator tiltVal", tiltVal);
   frc::SmartDashboard::PutNumber("Elevator armPos", armPos);

   //Elevator height
   frc::SmartDashboard::PutBoolean("ELevator Height limit switch", ls_vertElevator.Get());
   frc::SmartDashboard::PutNumber("Elevator Height Encoder", re_vertElevator.GetPosition());

   //Elevator tilt
   frc::SmartDashboard::PutBoolean("Elevator Tilt limit switch", ls_tiltElevator.Get());
   frc::SmartDashboard::PutNumber("Elevator Tilt Encoder", re_tiltElevator.GetPosition());
   frc::SmartDashboard::PutNumber("Elevator Tilt Output",  m_tiltElevatorMotor.Get());

   // //Elevator arm
   frc::SmartDashboard::PutBoolean("Elevator Arm limit switch", ls_arm.Get());
   frc::SmartDashboard::PutNumber("Elevator Arm encoder", re_arm.GetPosition());
   double curkPArm = frc::SmartDashboard::GetNumber("Elevator Arm kp", ElevatorConstants::kPModuleArmController);
   m_armPIDController.SetP(curkPArm);  
   double curKpVert = frc::SmartDashboard::GetNumber("Elevator Vert kp", ElevatorConstants::kPModuleVertController);
   m_vertPIDController.SetP(curKpVert);

   frc::SmartDashboard::PutNumber("Elevator state", ElevatorState);

   if (ElevatorState == FIND_ZERO_VERT){
      //limit switch is were the elevator is closest to the ground 
      m_vertElevatorMotorLeft.Set(-0.1);//go to reverse limit switch

      //limit switch is when the arm is all the way up
      m_armMotor.Set(0.05);//go to forward limit switch

      if (ls_arm.Get() == true){
         re_arm.SetPosition(0);
      }
      
      if((ls_vertElevator.Get() == true)) { 
         re_vertElevator.SetPosition(0);
         ElevatorState = FIND_ZERO_TILT; 
      } 

   } else if (ElevatorState == FIND_ZERO_TILT) {
      //limit switch is where the elevator is all the way tilted towards the back of the robot
      m_tiltElevatorMotor.Set(-0.2);//go to reverse limit switch

      //limit switch is when the arm is all the way up
      m_armMotor.Set(0.05);//go to forward limit switch

      if (ls_arm.Get() == true){
         re_arm.SetPosition(0);
      }

      if((ls_tiltElevator.Get() == true)) { 
         re_tiltElevator.SetPosition(0);
         //frc::SmartDashboard::PutBoolean("Elevator Reset Elevator Finished", true); //for debugging
         ElevatorState = MANUAL_MODE; 
      }


   } else if (ElevatorState == MANUAL_MODE){
      static double lastArmPos = 0.0;
      armMaxChange = frc::SmartDashboard::GetNumber("Elevator arm max change", armMaxChange);

      //arm speed limiter
      if (((armPos - lastArmPos) > armMaxChange)) { 
         armPos = lastArmPos + armMaxChange;
      } else if (((armPos - lastArmPos) < -armMaxChange)) {
         armPos = lastArmPos - armMaxChange;
      } 
      lastArmPos = armPos;

      //armPos safety
      if (armPos > 10){
         armPos = 10;
      } else if (armPos < -180){
         armPos = -180;
      }

      armOutput = m_armPIDController.Calculate(re_arm.GetPosition(), armPos);
      frc::SmartDashboard::PutNumber("Elevator armOutput", armOutput);

      static double lastVerticalPos = 0.0;
      vertMaxChange = frc::SmartDashboard::GetNumber("Elevator vert max change", vertMaxChange);

      /*if (re_tiltElevator.GetPosition() < 100){//need to change the number 50
         safeForVertElevator = false;
      } else if (re_tiltElevator.GetPosition() > 100){
         safeForVertElevator = true;
      }
      frc::SmartDashboard::PutBoolean("Elevator safe Up and Down", safeForVertElevator);*/

      //vertical elevator speed limiter
      if (((verticalPos - lastVerticalPos) > vertMaxChange)) { 
         verticalPos = lastVerticalPos + vertMaxChange;
      } else if (((verticalPos - lastVerticalPos) < -vertMaxChange)) {
         verticalPos = lastVerticalPos - vertMaxChange;
      } 
      lastVerticalPos = verticalPos;
      

      //verticalPos safety
      if (verticalPos < -1){
         verticalPos = -1;
      } else if (verticalPos > 106){
         verticalPos = 106;
      }
      
      vertOutput = m_vertPIDController.Calculate(re_vertElevator.GetPosition(), verticalPos);
      frc::SmartDashboard::PutNumber("Elevator vertOutput", vertOutput);

      //experimentally tested that a positive motor output of 0.037 made the output hold steady at -90 degrees
      m_armMotor.Set(armOutput); 
      m_tiltElevatorMotor.Set(tiltVal);
      m_vertElevatorMotorLeft.Set(vertOutput); //when use PID loop, change verticalVal to vertOutput

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
      verticalPos = 104;
      armPos = -60;
      
      static double lastArmPos = 0.0;
      armMaxChange = frc::SmartDashboard::GetNumber("Elevator arm max change", armMaxChange);

      //arm speed limiter
      if (((armPos - lastArmPos) > armMaxChange)) { 
         armPos = lastArmPos + armMaxChange;
      } else if (((armPos - lastArmPos) < -armMaxChange)) {
         armPos = lastArmPos - armMaxChange;
      } 
      lastArmPos = armPos;

      //armPos safety
      if (armPos > 10){
         armPos = 10;
      } else if (armPos < -180){
         armPos = -180;
      }

      armOutput = m_armPIDController.Calculate(re_arm.GetPosition(), armPos);
      frc::SmartDashboard::PutNumber("Elevator armOutput", armOutput);
      m_armMotor.Set(armOutput); 


      m_tiltElevatorMotor.Set(0.25);

      if(re_tiltElevator.GetPosition() >= 160){
         ElevatorState = MANUAL_MODE;
      }
   } else if (ElevatorState == PLACE_MID){

   } else if (ElevatorState == PLACE_LOW){

   }
}


void Elevator::ElevatorVert(double elevatorUp, double elevatorDown) { 
   frc::SmartDashboard::PutNumber("ElevatorUp Value", elevatorUp);
   frc::SmartDashboard::PutNumber("ElevatorDown Value", elevatorDown);
   frc::SmartDashboard::PutNumber("Elevator verticalVal", verticalVal);

   verticalVal =elevatorUp - elevatorDown;

   /*if ((fabs(verticalVal) < ElevatorConstants::vertDeadzone) && (enableElevator == true)){
      verticalVal = 0;
   }*/
   //if (safeForVertElevator == true){
      //for pid loop
      if (fabs(verticalVal) < ElevatorConstants::vertDeadzone && (enableElevator == true)) {
         verticalPos = verticalPos; //the arm stays in the same position
      } else if (enableElevator == true) {
         verticalPos = verticalPos + (verticalVal * (1.5));
      } else {
         verticalPos = 0;
      }
   /*} else {
      verticalPos = verticalPos;
   }*/

   frc::SmartDashboard::PutNumber("Elevator vertOutput", vertOutput);
}

void Elevator::ElevatorTilt(double lean){
   if ((fabs(lean) < ElevatorConstants::tiltDeadzone) && (enableElevator == true)){
      tiltVal = 0;
   } else if (enableElevator == true){
      tiltVal = -lean;
   } else {
      tiltVal = 0;
   }


}
void Elevator::ElevatorSetTiltOveride(double overide){
 tiltVal = overide;
}

void Elevator::ElevatorArm(double armXboxVal){

   // armMaxChange = frc::SmartDashboard::GetNumber("Elevator arm max change", armMaxChange);
   //static double lastArmPos = 0.0;

   if ((fabs(armXboxVal) < ElevatorConstants::armDeadzone) && (enableElevator == true)) {
      armPos = armPos; //the arm stays in the same position
   }/* else if (((armPos - lastArmPos) > armMaxChange) && (enableElevator == true)) { 
      //added maxChange part
      armPos = lastArmPos + armMaxChange;
   } else if (((armPos - lastArmPos) < -armMaxChange) && (enableElevator == true)) {
      armPos = lastArmPos - armMaxChange;
   } */else if (enableElevator == true){
      armPos = armPos + (armXboxVal * (2.0));
   } else {
      armPos = 0;
   }
   //lastArmPos = armPos;
}

void Elevator::SafeArm(){
   if (elevatorDisable == true){
      safeArmPos = 0;
   } else {
      static double lastArmPos = 0.0;

      //arm speed limiter
      if (((armPos - lastArmPos) > armMaxChange)) { 
         safeArmPos = lastArmPos + armMaxChange;
      } else if (((armPos - lastArmPos) < -armMaxChange)) {
         safeArmPos = lastArmPos - armMaxChange;
      } 
      lastArmPos = safeArmPos;

      //armPos safety
      if (armPos > 10){
         safeArmPos = 10;
      } else if (armPos < -180){
         safeArmPos = -180;
      }
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
   return this->Run(
      [this, angle] { armPos = angle; });
}

frc2::CommandPtr Elevator::SetVertPos(double revolutions){
   return this->Run(
      [this, revolutions] { verticalPos = revolutions; });
}

frc2::CommandPtr Elevator::SetElevatorPos(double armAngle, double vertRevolutions){
   return this->Run(
      [this, armAngle, vertRevolutions] {
         armPos = armAngle; 
         verticalPos = vertRevolutions; }
   );
}

frc2::CommandPtr Elevator::SetTiltElevator(double velocity){
   return this->Run(
      [this, velocity] { m_tiltElevatorMotor.Set(velocity); });
}

double Elevator::TiltEncoderValues(){
   return re_tiltElevator.GetPosition();
}

double Elevator::HeightEncoderValues(){
   return re_vertElevator.GetPosition();
}

double Elevator::ArmEncoderValues(){
   return re_arm.GetPosition();
}

void Elevator::AutoPlace(double armAngle, double vertRevolutions){
   armPos = armAngle; 
   verticalPos = vertRevolutions;
}

//this is if Ian wants an elevator button on the triggers of his xbox
/*void Elevator::TriggerButtons(double leftTrigger, double rightTrigger){
   if (leftTrigger > 0.15){
      armPos = -88;
      verticalPos = 5;
      //m_tiltElevatorMotor.Set();
        double m_actualTiltVelocity;
  if(IsClose(m_tiltRevolutions, m_elevator->TiltEncoderValues(), tiltTolerance)){
    tiltVal = 0;
  }
  else if(fabs(m_tiltRevolutions - m_elevator->TiltEncoderValues()) > 15){
    tiltVal = m_tiltVelocity;
  }
  else{
    tiltVal = m_tiltVelocity/2;
  }

   } else if (rightTrigger > 0.15){
      armPos = -5;
   }
}
bool Elevator::IsClose(double check1, double check2, double thresh){
  if(fabs(check1 - check2) < thresh){
    return true;
  }
  else{
    return false;
  }
}*/