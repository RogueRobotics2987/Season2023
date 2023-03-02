// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoBalance.h"

AutoBalance::AutoBalance(DriveSubsystem& l_drive, frc::XboxController &l_Joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_drive = &l_drive;
  m_Joystick = &l_Joystick;
  AddRequirements({m_drive});
}

AutoBalance::AutoBalance(){}
// Called when the command is initially scheduled.
void AutoBalance::Initialize() {
  m_state = 0;
  backwardsCheck = false;
  m_timer.Stop();
  m_timer.Reset();
  // autoCheck = m_Joystick->GetRawButton(11);
// Called repea  
  // m_angle = 9;
  // frc::SmartDashboard::PutNumber("m_angle value: ", m_angle);
}

// Called repeatedly when this Command is scheduled to run
void AutoBalance::Execute() {
  frc::SmartDashboard::PutNumber("Auto State", m_state);
  // frc::SmartDashboard::PutNumber("Auto Pitch", m_drive->GetPitch());
  frc::SmartDashboard::PutNumber("Auto Timer", m_timer.Get().value());
  if(m_state == 0) { //Rams charge station
    // m_angle = frc::SmartDashboard::GetNumber("m_angle value: ", m_angle);
      m_drive->Drive(0_mps, 2.0_mps, 0_rad_per_s, false, false);
      if(m_drive->GetPitch() >= 9 || m_drive->GetPitch() <= -9) {
        m_state = 1;
      }
      
    }
    else if(m_state == 1){ //Slower climb
      m_drive->Drive(0_mps, 0.45_mps, 0_rad_per_s, false, false);
      // if(m_drive->GetPitch() <= 4.5 && m_drive->GetPitch() >= -4.5) {
      if(m_drive->GetRawGyroX() <-3) { //Degrees per second, clock hand is 6 degrees per second
        m_state = 3;
        m_timer.Start();
      }
      
    }
    // if(m_state == 2) { //Timer/stop state
    //   if(m_drive->GetPitch() <= 9 && backwardsCheck == false) {
    //     m_state = 3;
    //     m_timer.Start();
    //     backwardsCheck = true;
    //   }
    //   else if(m_drive->GetPitch() < 10 && m_drive->GetPitch() > -10 && m_timer.Get() > 5.0_s) {
    //       m_state = 5;
    //     }
    //     else if(m_timer.Get() <= 5_s && m_drive->GetPitch() < 10 && m_drive->GetPitch() > -10) {
    //       m_drive->Drive(0.0_mps, 0_mps, 0_rad_per_s, false, true);
    //     }
    //     else {
    //       m_state = 4;
    //       m_timer.Reset();
    //       m_timer.Stop();
    //     }
      // else if(m_drive->GetPitch() > 1 || m_drive->GetPitch() < -1) {
      //   m_state = 4;
      //   m_timer.Stop();
      // }
      // else if(m_drive->GetPitch() < 1 && m_drive->GetPitch() > -1 && m_timer.Get() >= 5_s) {
      //   m_state = 5;
      // }
      
    // }
      else if(m_state == 3) { //Run backwards
        if(m_timer.Get() <= 0.75_s) {
          m_drive->Drive(0_mps, -0.75_mps, 0_rad_per_s, false, false);
        }
        else {
          m_state = 5;
          // m_timer.Reset();
          // m_timer.Stop();
        }
      // if(m_drive->GetPitch() <= 7.75) {
      //   m_state = 2;
      // }
    }
      // if(m_state == 4) { //Auto Balance state
      //   if(m_drive->GetPitch() < 10 && m_drive->GetPitch() > -10) {
      //     m_state = 2;
      //     m_timer.Start();
      //   }
      //   else if(m_drive->GetPitch() <= -10) {
      //     m_drive->Drive(0_mps, -0.35_mps, 0_rad_per_s, false, false);
      //   }
      //   else if(m_drive->GetPitch() >= 10) {
      //     m_drive->Drive(0_mps, 0.35_mps, 0_rad_per_s, false, false);
      //   }
      // }
      if(m_state == 5){ //Exit state
        m_drive->Drive(0_mps, 0.0_mps, 0_rad_per_s, false, false);
      }
      // while(m_state == 5) {
      //   frc::SmartDashboard::PutNumber("Auto State", m_state);
      //   frc::SmartDashboard::PutNumber("Auto Pitch", m_drive->GetPitch());
      //   frc::SmartDashboard::PutNumber("Auto Timer", m_timer.Get().value());
      //   m_drive->Drive(0.0_mps, 0_mps, 0_rad_per_s, false, true);
      //   if(m_drive->GetPitch() < 1 && m_drive->GetPitch() > -1 && m_timer.Get() > 5_s) {
      //     m_state = 6;
      //   }
      //   else if(m_timer.Get() <= 5_s && m_drive->GetPitch() < 1 && m_drive->GetPitch() > -1) {
      //     m_drive->Drive(0.0_mps, 0_mps, 0_rad_per_s, false, true);
      //   }
      //   else {
      //     m_state = 4;
      //     m_timer.Reset();
      //     m_timer.Stop();
      //   }
      // }

// if(curAngle >= 1 || curAngle <= -1) {
//   m_drive->Drive(-speed, 0_mps, 0_rad_per_s, false, false);
// } else {
//   m_drive->Drive(0.0_mps, 0_mps, 0_rad_per_s, false, false);
// }
// if(!m_Joystick->GetRawButton(11) && autoCheck){
//   m_state = 5;
// }
}
    //while(m_state == 2) {
      // m_drive->Drive(0.0_mps, 0_mps, 0_rad_per_s, false, false);
    //}

// frc2::CommandPtr AutoBalance::runCmd(bool run){
//   return this->RunOnce(
//     [this, run] {m_runCmd = run; });
// }

void AutoBalance::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoBalance::IsFinished() {
  if(m_state == 5){
    return true;
  }
  else{
    return false;
  }
}
