// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"

// for limelight, configOdometry (temp?)
#include <iostream>
#include <span>
using namespace std;

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : 
    
    // m_frontLeft{kFrontLeftDriveMotorPort,
    //               kFrontLeftTurningMotorPort,
    //               kFrontLeftDriveEncoderPorts,
    //               kFrontLeftTurningEncoderPorts,
    //               kFrontLeftDriveEncoderReversed,
    //               kFrontLeftTurningEncoderReversed},

    //   m_rearLeft{
    //       kRearLeftDriveMotorPort,       kRearLeftTurningMotorPort,
    //       kRearLeftDriveEncoderPorts,    kRearLeftTurningEncoderPorts,
    //       kRearLeftDriveEncoderReversed, kRearLeftTurningEncoderReversed},

    //   m_frontRight{
    //       kFrontRightDriveMotorPort,       kFrontRightTurningMotorPort,
    //       kFrontRightDriveEncoderPorts,    kFrontRightTurningEncoderPorts,
    //       kFrontRightDriveEncoderReversed, kFrontRightTurningEncoderReversed},

    //   m_rearRight{
    //       kRearRightDriveMotorPort,       kRearRightTurningMotorPort,
    //       kRearRightDriveEncoderPorts,    kRearRightTurningEncoderPorts,
    //       kRearRightDriveEncoderReversed, kRearRightTurningEncoderReversed},

    m_frontLeft{
        kFrontLeftDriveMotorPort, m_EncoderType, kFrontLeftDriveCPR, 
        kFrontLeftTurningMotorPort, 
        kFrontLeftDriveEncoderReversed, 
        kFrontLeftTurningEncoderNumber,
        kFrontLeftTurningEncoderReversed
      },
      m_frontRight{
        kFrontRightDriveMotorPort, m_EncoderType, kFrontRightDriveCPR, 
        kFrontRightTurningMotorPort,
        kFrontRightDriveEncoderReversed, 
        kFrontRightTurningEncoderNumber,
        kFrontRightTurningEncoderReversed
      },
      m_rearLeft{
        kRearLeftDriveMotorPort, m_EncoderType, kRearLeftDriveCPR, 
        kRearLeftTurningMotorPort, 
        kRearLeftDriveEncoderReversed, 
        kRearLeftTurningEncoderNumber,
        kRearLeftTurningEncoderReversed
      },
      m_rearRight{
        kRearRightDriveMotorPort, m_EncoderType, kRearRightDriveCPR, 
        kRearRightTurningMotorPort, 
        kRearRightDriveEncoderReversed, 
        kRearRightTurningEncoderNumber,
        kRearRightTurningEncoderReversed
      },


      m_odometry{kDriveKinematics,
                 m_gyro.GetRotation2d(),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}} 
                 
{frc::SmartDashboard::PutData("Field", &m_field);}

frc2::CommandPtr DriveSubsystem::SetDriveSlow(bool m_bool){
  return this->RunOnce(
    [this, m_bool] {driveSlow = m_bool; });
}

void DriveSubsystem::Periodic() {
  // commented out to test, 2/17
  frc::SmartDashboard::PutNumber("Gyro Yaw: ", m_gyro.GetYaw());
  frc::SmartDashboard::PutNumber("Gyro Pitch: ", m_gyro.GetPitch());
  frc::SmartDashboard::PutNumber("Gyro Roll: ", m_gyro.GetRoll());
  frc::SmartDashboard::PutNumber("Gyro Velocity X", m_gyro.GetRawGyroX());
  // frc::SmartDashboard::PutNumber("Gyro Angle X: ", m_gyro.GetRawGyroX());
  // frc::SmartDashboard::PutNumber("Gyro Angle Y: ", m_gyro.GetRawGyroY());
  // frc::SmartDashboard::PutNumber("Gyro Angle Z: ", m_gyro.GetRawGyroZ());

  frc::SmartDashboard::PutNumber("Odometry Rotation", m_odometry.GetPose().Rotation().Degrees().value());
  frc::SmartDashboard::PutNumber("Odometry X", m_odometry.GetPose().X().value());
  frc::SmartDashboard::PutNumber("Odometry Y", m_odometry.GetPose().Y().value());




  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});
    // commented out to test, 2/17
    frc::SmartDashboard::PutNumber("NavX Heading: ", (double)m_gyro.GetRotation2d().Degrees());

  m_field.SetRobotPose(m_odometry.GetPose());
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative,
                           bool noJoystick) {
                      
  // commented out to test, 2/17
  //frc::SmartDashboard::PutNumber("ROT value: ", rot.value());
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  // for(int i = 0; i<4; i++){
  //   states[i].speed * ModuleConstants::kFFModuleDriveController;
  // }

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  
  float currentAngle = fmod((double)(m_frontLeft.GetState().angle.Degrees()),180);
  // float desiredAngle = (float)fl.angle.Degrees();
  // TODO curentAngle = currentAngle + 89.65 Radients moduleConstants::Wheelconstants
  //float angleDiffFL = fabs((float)(fl.angle.Degrees()) - currentAngle);
  //float angleOff = fabs(currentAngle - desiredAngle);
  float angleOff = (float)m_frontLeft.GetTurnPID().GetPositionError();
  
  // commented out to test, 2/17
  // frc::SmartDashboard::PutNumber("Fl Current angle", currentAngle);
  // frc::SmartDashboard::PutNumber("Fl Angle Diff",angleOff);
  // // frc::SmartDashboard::PutNumber("m_frontLeft State Angle", m_frontLeft.GetState().angle.Degrees().value());
  // frc::SmartDashboard::PutNumber("Fl Desired angle",(float)fl.angle.Degrees());
  // frc::SmartDashboard::PutNumber("Fr Desired angle",(float)fr.angle.Degrees());
  // frc::SmartDashboard::PutNumber("Bl Desired angle",(float)bl.angle.Degrees());
  // frc::SmartDashboard::PutNumber("Br Desired angle",(float)br.angle.Degrees());
  float epsilon = 1.0/10.0;
  // angleOff = angleOff && fabs((angleDiffFR < 10)) && (!noJoystick);
  // angleOff = angleOff && fabs((angleDiffBR < 10)) && (!noJoystick);
  // angleOff = angleOff && fabs((angleDiffBL < 10)) && (!noJoystick);
  if(fl.speed > (units::velocity::meters_per_second_t)(0.05)){
    
  }

  else if(fabs(angleOff) <= epsilon && noJoystick != true) {
    // m_frontLeft.SetDesiredState(fl);
    // m_frontRight.SetDesiredState(fr);
    // m_rearLeft.SetDesiredState(bl);
    // m_rearRight.SetDesiredState(br);    
  }
  else if(noJoystick){
    fl.speed = (units::velocity::meters_per_second_t)(0);
    fr.speed = (units::velocity::meters_per_second_t)(0);
    bl.speed = (units::velocity::meters_per_second_t)(0);
    br.speed = (units::velocity::meters_per_second_t)(0);
   fl.angle = (units::angle::degree_t)(45);
    fr.angle = (units::angle::degree_t)(135);
    bl.angle = (units::angle::degree_t)(-45);
    br.angle = (units::angle::degree_t)(-135); 
    // m_frontLeft.SetDesiredState(fl);
    // m_frontRight.SetDesiredState(fr);
    // m_rearLeft.SetDesiredState(bl);
    // m_rearRight.SetDesiredState(br);
  }
  else{
    fl.speed = (units::velocity::meters_per_second_t)(0);
    fr.speed = (units::velocity::meters_per_second_t)(0);
    bl.speed = (units::velocity::meters_per_second_t)(0);
    br.speed = (units::velocity::meters_per_second_t)(0);
  }

  if(driveSlow == true){
    fl.speed = (units::velocity::meters_per_second_t)(0.5 * fl.speed);
    fr.speed = (units::velocity::meters_per_second_t)(0.5 * fr.speed);
    bl.speed = (units::velocity::meters_per_second_t)(0.5 * bl.speed);
    br.speed = (units::velocity::meters_per_second_t)(0.5 * br.speed);
  }
  if(WheelsStraight == true){
    fl.angle = (units::angle::degree_t)(0);
    fr.angle = (units::angle::degree_t)(0);
    bl.angle = (units::angle::degree_t)(0);
    br.angle = (units::angle::degree_t)(0);
  }
  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::DriveAutonomous(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative,
                           bool noJoystick) {
                      
  // commented out to test, 2/17
  //frc::SmartDashboard::PutNumber("ROT value: ", rot.value());
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  // for(int i = 0; i<4; i++){
  //   states[i].speed * ModuleConstants::kFFModuleDriveController;
  // }

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  
  float currentAngle = fmod((double)(m_frontLeft.GetState().angle.Degrees()),180);
  // float desiredAngle = (float)fl.angle.Degrees();
  // TODO curentAngle = currentAngle + 89.65 Radients moduleConstants::Wheelconstants
  //float angleDiffFL = fabs((float)(fl.angle.Degrees()) - currentAngle);
  //float angleOff = fabs(currentAngle - desiredAngle);
  float angleOff = (float)m_frontLeft.GetTurnPID().GetPositionError();
  
  // commented out to test, 2/17
  // frc::SmartDashboard::PutNumber("Fl Current angle", currentAngle);
  // frc::SmartDashboard::PutNumber("Fl Angle Diff",angleOff);
  // // frc::SmartDashboard::PutNumber("m_frontLeft State Angle", m_frontLeft.GetState().angle.Degrees().value());
  // frc::SmartDashboard::PutNumber("Fl Desired angle",(float)fl.angle.Degrees());
  // frc::SmartDashboard::PutNumber("Fr Desired angle",(float)fr.angle.Degrees());
  // frc::SmartDashboard::PutNumber("Bl Desired angle",(float)bl.angle.Degrees());
  // frc::SmartDashboard::PutNumber("Br Desired angle",(float)br.angle.Degrees());
  float epsilon = 1.0/10.0;
  // angleOff = angleOff && fabs((angleDiffFR < 10)) && (!noJoystick);
  // angleOff = angleOff && fabs((angleDiffBR < 10)) && (!noJoystick);
  // angleOff = angleOff && fabs((angleDiffBL < 10)) && (!noJoystick);

  // if(fl.speed > (units::velocity::meters_per_second_t)(0.05)){
    
  // }

  // else if(fabs(angleOff) <= epsilon && noJoystick != true) {
  //   // m_frontLeft.SetDesiredState(fl);
  //   // m_frontRight.SetDesiredState(fr);
  //   // m_rearLeft.SetDesiredState(bl);
  //   // m_rearRight.SetDesiredState(br);    
  // }
  // else if(noJoystick){
  //   fl.speed = (units::velocity::meters_per_second_t)(0);
  //   fr.speed = (units::velocity::meters_per_second_t)(0);
  //   bl.speed = (units::velocity::meters_per_second_t)(0);
  //   br.speed = (units::velocity::meters_per_second_t)(0);
  //  fl.angle = (units::angle::degree_t)(45);
  //   fr.angle = (units::angle::degree_t)(135);
  //   bl.angle = (units::angle::degree_t)(-45);
  //   br.angle = (units::angle::degree_t)(-135); 
  //   // m_frontLeft.SetDesiredState(fl);
  //   // m_frontRight.SetDesiredState(fr);
  //   // m_rearLeft.SetDesiredState(bl);
  //   // m_rearRight.SetDesiredState(br);
  // }
  // else{
  //   fl.speed = (units::velocity::meters_per_second_t)(0);
  //   fr.speed = (units::velocity::meters_per_second_t)(0);
  //   bl.speed = (units::velocity::meters_per_second_t)(0);
  //   br.speed = (units::velocity::meters_per_second_t)(0);
  // }

  // if(driveSlow == true){
  //   fl.speed = (units::velocity::meters_per_second_t)(0.5 * fl.speed);
  //   fr.speed = (units::velocity::meters_per_second_t)(0.5 * fr.speed);
  //   bl.speed = (units::velocity::meters_per_second_t)(0.5 * bl.speed);
  //   br.speed = (units::velocity::meters_per_second_t)(0.5 * br.speed);
  // }
  // if(WheelsStraight == true){
  //   fl.angle = (units::angle::degree_t)(0);
  //   fr.angle = (units::angle::degree_t)(0);
  //   bl.angle = (units::angle::degree_t)(0);
  //   br.angle = (units::angle::degree_t)(0);
  // }
  
  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, AutoConstants::kMaxSpeed);
  
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_gyro.GetRotation2d().Degrees();
}

float DriveSubsystem::GetPitch() {
  return m_gyro.GetPitch();
  }

float DriveSubsystem::GetRoll(){
  return m_gyro.GetRoll();
}

float DriveSubsystem::GetRawGyroX(){
  return m_gyro.GetRawGyroX();
}


frc2::CommandPtr DriveSubsystem::ZeroHeading() {
  return this->RunOnce(
    [this] {
      m_gyro.Reset();
    });
}

frc2::CommandPtr DriveSubsystem::SetAngleAdjustment(double angle){
  return this->RunOnce(
    [this, angle] {
      m_gyro.SetAngleAdjustment(angle);
    });
}

frc2::CommandPtr DriveSubsystem::FieldOrientatedTrue() { //idk
   return this->RunOnce(
      [this] { fieldOrientated = true;
      frc::SmartDashboard::PutBoolean("Field orientated control", fieldOrientated); });
}

frc2::CommandPtr DriveSubsystem::FieldOrientatedFalse() { //idk
   return this->RunOnce(
      [this] { fieldOrientated = false;
      frc::SmartDashboard::PutBoolean("Field orientated control", fieldOrientated); });
}

double DriveSubsystem::GetTurnRate() {
  return -m_gyro.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}

frc2::CommandPtr DriveSubsystem::ButtonZeroHeading(){
  return this->RunOnce(
    [this] {ZeroHeading();});
}

void DriveSubsystem::ConfigMotorControllers(){
  m_frontLeft.ConfigMotorControllers();
  m_frontRight.ConfigMotorControllers();
  m_rearLeft.ConfigMotorControllers();
  m_rearRight.ConfigMotorControllers();
}

frc2::CommandPtr DriveSubsystem::ConfigOdometry(){
  return this ->RunOnce( [this] {
    double default_array[6] = {};
    //numAT = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0); // double check, value might only reconize if it sees something, not how many in total

    std::vector<double> botpose = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->
                                  GetNumberArray("botpose", std::span{default_array, std::size(default_array)});
  
    for(int i=0; i<6; i++){
      cout << botpose[i] << ", ";
    }

    /*if(numAT > 1){
      cout << numAT << endl;
    } else {
      cout << numAT << endl;
    }*/

    frc::Pose2d pose(units::meter_t(botpose.at(0)), units::meter_t(botpose.at(1)), units::radian_t(botpose.at(5)));
    ResetOdometry(pose);

  });
}

frc2::CommandPtr DriveSubsystem::Twitch(bool direction){
  return this -> Run(
    [this, direction]{
  if(direction == true){
    //right
    DriveSubsystem::Drive(0_mps, -0.35_mps, 0_rad_per_s, false, false);
    
  }
  else if(direction == false){
    //left
    DriveSubsystem::Drive(0_mps, 0.35_mps, 0_rad_per_s, false, false);
  }
  else{
    //stop
    DriveSubsystem::Drive(0_mps, 0.0_mps, 0_rad_per_s, false, false);
  }
  });
}