// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// #include "commands/InPlaceRotationCmd.h"

// InPlaceRotationCmd::InPlaceRotationCmd(double angle) {
//   // Use addRequirements() here to declare subsystem dependencies.
//   if((m_drive.GetHeading() + angle) > 180){
//     this->m_angle = (m_drive.GetHeading() + angle) - 360;
//   }
//   else if((m_drive.GetHeading() + angle) < -180){
//     this->m_angle = (m_drive.GetHeading() + angle) + 360;
//   }
//   else{
//     this->m_angle = (m_drive.GetHeading() + angle);
//   }  
// }

// // Called when the command is initially scheduled.
// //move all into constructor
// void InPlaceRotationCmd::Initialize() {

// }

// // Called repeatedly when this Command is scheduled to run
// void InPlaceRotationCmd::Execute() {
//   // if(m_angle - ){

//   // }
// }

// // Called once the command ends or is interrupted.
// void InPlaceRotationCmd::End(bool interrupted) {}

// // Returns true when the command should end.
// bool InPlaceRotationCmd::IsFinished() {
//   return false;
// }

// double DistanceBetweenAngles(double angle1, double angle2){
//   //TODO double check else ifs they may not be correct 
//   //if angle 1 is pos and angle 2 is neg
//   if(angle1 > 0 && angle2 < 0){
//     return 360 - (angle1 - angle2);
//   }
//   //if angle 1 is neg and 2 is pos
//   else if(angle1 < 0 && angle2 > 0){
//   //both neg
//     // return 360 + (angle1 - angle2); 
//   }
//   else if(angle1 < 0 && angle2 < 0){
//   //both pos
//     // return angle1 - angle2;
//   }
//   else if(angle1 > 0 && angle2 > 0){
//     // return 360 + (angle1 - angle2);
//   }
// }
