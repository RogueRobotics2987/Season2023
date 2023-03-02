// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PlaceAutoCmd.h"

PlaceAutoCmd::PlaceAutoCmd(Elevator &elevator, double heightRevolutions, double armAngle, double tiltRevolutions) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_elevator = &elevator;
  AddRequirements({m_elevator});
  m_armAngle = armAngle; //goes 0 -180
  m_heightRevolutions = heightRevolutions; 
  m_tiltRevolutions = tiltRevolutions;
  // if(m_tiltRevolutions - m_elevator->TiltEncoderValues() < 0){
  //   m_tiltVelocity = -0.4;
  // }
  // else{
  //   m_tiltVelocity = 0.4;
  // }
  double PlaceCmdState;
  frc::SmartDashboard::PutString("PlaceCmdState", "Contrtuctor");
}


// Called when the command is initially scheduled.
void PlaceAutoCmd::Initialize() {
  frc::SmartDashboard::PutString("PlaceCmdState", "Initialize");
  if(m_tiltRevolutions - m_elevator->TiltEncoderValues() < 0){
    m_tiltVelocity = -0.8;
  }
  else{
    m_tiltVelocity = 0.8;
  }
}

// Called repeatedly when this Command is scheduled to run
void PlaceAutoCmd::Execute() {
  double m_actualTiltVelocity;
  if(m_tiltRevolutions - m_elevator->TiltEncoderValues() > 15){
    m_actualTiltVelocity = m_tiltVelocity;
  }
  else{
    m_actualTiltVelocity = m_tiltVelocity/2;
  }
  m_elevator->m_tiltElevatorMotor.Set(m_actualTiltVelocity);
  m_elevator->AutoPlace(m_armAngle, m_heightRevolutions);
  frc::SmartDashboard::PutString("PlaceCmdState", "Execute");

}

// Called once the command ends or is interrupted.
void PlaceAutoCmd::End(bool interrupted) {
  m_elevator->m_tiltElevatorMotor.Set(0.0);
  frc::SmartDashboard::PutString("PlaceCmdState", "End");
}

// Returns true when the command should end.
bool PlaceAutoCmd::IsFinished() {
  if(IsClose(m_elevator->TiltEncoderValues(), m_tiltRevolutions) && IsClose(m_elevator->ArmEncoderValues(), m_armAngle) && IsClose(m_elevator->HeightEncoderValues(), m_heightRevolutions)){
    return true;
  }else{
    return false;
  }
  frc::SmartDashboard::PutString("PlaceCmdState", "Finished");

}

bool PlaceAutoCmd::IsClose(double check1, double check2){
  if(fabs(check1 - check2) < 5){
    return true;
  }
  else{
    return false;
  }
}