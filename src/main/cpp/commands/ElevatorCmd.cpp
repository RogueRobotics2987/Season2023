// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ElevatorCmd.h"

ElevatorCmd::ElevatorCmd(){}
ElevatorCmd::ElevatorCmd(Elevator &elevator, frc::XboxController &xbox, frc::XboxController &newXbox) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_elevator = &elevator;
  m_xbox = &xbox;
  m_newXbox = &newXbox;
  AddRequirements({m_elevator});

}

// Called when the command is initially scheduled.
void ElevatorCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ElevatorCmd::Execute() {
  //right trigger (3) =  up
  //left trigger (2) = down
  // commented out to test, 2/17 frc::SmartDashboard::PutNumber("ELevator up", m_xbox->GetRawAxis(3));
  m_elevator->ElevatorVert(m_xbox->GetRawAxis(3), m_xbox->GetRawAxis(2));
  m_elevator->ElevatorTilt(m_xbox->GetRawAxis(1)); //Y axis of the left joystick on the xbox controller
  m_elevator->ElevatorArm(m_xbox->GetRawAxis(5)); //Y axis of the right joystick on the xbox controller
  frc::SmartDashboard::PutNumber("Elevator arm xbox value", m_xbox->GetRawAxis(5));
  //m_elevator->TriggerButtons(m_newXbox->GetRawAxis(2), m_elevator->GetRawAxis(3));
  
}

// Called once the command ends or is interrupted.
void ElevatorCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool ElevatorCmd::IsFinished() {
  return false;
}

