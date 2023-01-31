// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator() {
   m_vertElevatorMotorLeft.SetInverted(false);
   m_vertElevatorMotorRight.SetInverted(false);
   m_tiltElevatorMotor.SetInverted(false);
   m_clawMotor.SetInverted(false);
}

// This method will be called once per scheduler run
void Elevator::Periodic() {
   frc::SmartDashboard::PutNumber("verticalVal", verticalVal);

   m_vertElevatorMotorLeft.Set(verticalVal);
   m_vertElevatorMotorRight.Set(verticalVal);
}

   frc2::CommandPtr Elevator::MotorMoveCommand() {
      return this->RunOnce(
      [this] { m_clawMotor.Set(0.5); });
   }

   frc2::CommandPtr Elevator::StopMoveCommand(){
      return this->RunOnce(
         [this] {m_clawMotor.Set(0); });
   }


   void Elevator::ElevatorVert(double elevatorUp, double elevatorDown) {
      frc::SmartDashboard::PutNumber("elevatorUp Value", elevatorUp);
      frc::SmartDashboard::PutNumber("elevatorDown Value", elevatorDown);

      if((fabs(elevatorUp) > upDeadzone) && (elevatorDown < downDeadzone) && (enableElevatorVert == true)){
        verticalVal = elevatorUp - upDeadzone;
      } else if((fabs(elevatorDown) > downDeadzone) && (elevatorUp < upDeadzone) && (enableElevatorVert == true)){
        verticalVal = -(elevatorDown - downDeadzone);
      } else {
        verticalVal = 0;
      }
   }