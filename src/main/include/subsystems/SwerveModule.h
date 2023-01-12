// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"

#include "Constants.h"

class SwerveModule {
 public:
//   SwerveModule(int driveMotorChannel, int turningMotorChannel,
//                const int driveEncoderPorts[2], const int turningEncoderPorts[2],
//                bool driveEncoderReversed, bool turningEncoderReversed);


SwerveModule(int m_MotorController, rev::SparkMaxRelativeEncoder::Type m_EncoderType, int m_counts_per_rev, 
      int m_MotorControllerTurning, 
      bool driveEncoderReversed,
      int TurningEncoderNumber,
      bool turningEncoderReversed
 );
 ~SwerveModule();

  frc::SwerveModuleState GetState();

  frc::SwerveModulePosition GetPosition();

  void SetDesiredState(const frc::SwerveModuleState& state);

  void ResetEncoders();

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr auto kModuleMaxAngularVelocity =
      units::radians_per_second_t{std::numbers::pi * 8.0};
  static constexpr auto kModuleMaxAngularAcceleration =
      units::radians_per_second_squared_t{std::numbers::pi * 16.0};

  rev::CANSparkMax* m_driveMotor;
  rev::CANSparkMax* m_turningMotor;

  rev::SparkMaxRelativeEncoder* m_driveEncoder;
  rev::SparkMaxRelativeEncoder::Type m_EncoderType;
  int m_counts_per_rev;

  ctre::phoenix::sensors::CANCoder* m_turningEncoder;

  bool m_reverseDriveEncoder;
  bool m_reverseTurningEncoder;

  frc2::PIDController m_drivePIDController{
      ModuleConstants::kPModuleDriveController, 0, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      ModuleConstants::kPModuleTurningController,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
};
