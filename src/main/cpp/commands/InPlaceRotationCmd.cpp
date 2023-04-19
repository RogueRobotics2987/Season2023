// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/InPlaceRotationCmd.h"

InPlaceRotationCmd::InPlaceRotationCmd(double angle, DriveSubsystem &drive) {
  //Angle must be within 180, -180
  m_drive = &drive;
  AddRequirements({m_drive});
  turn_amount = angle;
  frc::SmartDashboard::PutNumber("Rot KP auto", rot_kp);

}

// Called when the command is initially scheduled.
//move all into constructor
void InPlaceRotationCmd::Initialize() {

  if((m_drive->GetHeading().value() + turn_amount) > 180){
    this->m_angle = (m_drive->GetHeading().value() + turn_amount) - 360;
  }
  else if((m_drive->GetHeading().value() + turn_amount) < -180){
    this->m_angle = (m_drive->GetHeading().value() + turn_amount) + 360;
  }
  else{
    this->m_angle = (m_drive->GetHeading().value() + turn_amount);
  }  
}

// Called repeatedly when this Command is scheduled to run
void InPlaceRotationCmd::Execute() {

  m_dist = DistanceBetweenAngles(m_angle, m_drive->GetHeading().value());
  frc::SmartDashboard::PutNumber("InPlaceRot dist between des and cur", m_dist);
  rot_kp = frc::SmartDashboard::GetNumber("Rot KP auto", rot_kp);

  // //full speed when far
  // if(m_dist > outer_band){
  //   m_drive->Drive(0_mps, 0_mps, -max_rot_speed, false, true);
  // }
  // else if(m_dist < -outer_band){
  //   m_drive->Drive(0_mps, 0_mps, max_rot_speed, false, true);
  // }
  // //reducing speed when we get close
  // else if(m_dist <= outer_band && m_dist > inner_band){
  //   m_drive->Drive(0_mps, 0_mps, -mid_rot_speed, false, true);
  // }
  // else if(m_dist >= -outer_band && m_dist < -inner_band){
  //   m_drive->Drive(0_mps, 0_mps, mid_rot_speed, false, true);
  // }

  m_drive->Drive(0_mps, 0_mps, units::radians_per_second_t(-1 * (m_dist * rot_kp)), false, true);
}


// Called once the command ends or is interrupted.
void InPlaceRotationCmd::End(bool interrupted) {
  m_drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
}

// Returns true when the command should end.
bool InPlaceRotationCmd::IsFinished() {
  if(fabs(m_dist) <= inner_band){
    return true;
  }
  return false;
}

double InPlaceRotationCmd::DistanceBetweenAngles(double angle1, double angle2){
  //if angle 1 is pos and angle 2 is neg
  if(angle1 > 0 && angle2 < 0){
    return 360 - (angle1 - angle2);
  }
    //angle 1 is neg and 2 is pos
  else if(angle1 < 0 && angle2 > 0){
    return (angle2 - angle1) - 360;
  }
    //both neg
  else if(angle1 < 0 && angle2 < 0){
    return angle2 - angle1;
  }
    //both pos
  else if(angle1 > 0 && angle2 > 0){
    return angle2 - angle1;
  }
    //zeros
  else if(angle1 == 0){
    return angle2;
  }
  else if(angle2 == 0){
    return angle1;
  }
}
