// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"

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
                 frc::Pose2d{}} {}

frc2::CommandPtr DriveSubsystem::SetDriveSlow(bool m_bool){
  return this->RunOnce(
    [this, m_bool] {driveSlow = m_bool; });
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
                      
  frc::SmartDashboard::PutNumber("ROT value: ", rot.value());
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

float currentAngle = fabs(fmod((double)(m_frontLeft.GetState().angle.Degrees()),360));
// TODO curentAngle = currentAngle + 89.65 Radients moduleConstants::Wheelconstants
float angleDiff = fabs((float)(fl.angle.Degrees()) - currentAngle);
frc::SmartDashboard::PutNumber("Fl Current angle", currentAngle);
frc::SmartDashboard::PutNumber("Fl Angle Diff",angleDiff);
// frc::SmartDashboard::PutNumber("m_frontLeft State Angle", m_frontLeft.GetState().angle.Degrees().value());
frc::SmartDashboard::PutNumber("Fl Desired angle",(float)fl.angle.Degrees());
frc::SmartDashboard::PutNumber("Fr Desired angle",(float)fr.angle.Degrees());
frc::SmartDashboard::PutNumber("Bl Desired angle",(float)bl.angle.Degrees());
frc::SmartDashboard::PutNumber("Br Desired angle",(float)br.angle.Degrees());
if (angleDiff < 10 or fabs(angleDiff - 360) < 10){
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

    // m_frontLeft.SetDesiredState(fl);
    // m_frontRight.SetDesiredState(fr);
    // m_rearLeft.SetDesiredState(bl);
    // m_rearRight.SetDesiredState(br);
  }
  if(driveSlow == true){
    fl.speed = (units::velocity::meters_per_second_t)(0.5 * fl.speed);
    fr.speed = (units::velocity::meters_per_second_t)(0.5 * fr.speed);
    bl.speed = (units::velocity::meters_per_second_t)(0.5 * bl.speed);
    br.speed = (units::velocity::meters_per_second_t)(0.5 * br.speed);
  }
  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
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

void DriveSubsystem::ZeroHeading() {
  m_gyro.Reset();
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
