// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

#include "Constants.h"

// SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
//                            const int driveEncoderPorts[],
//                            const int turningEncoderPorts[],
//                            bool driveEncoderReversed,
//                            bool turningEncoderReversed)
//     : m_driveMotor(driveMotorChannel),
//       m_turningMotor(turningMotorChannel),
//       m_driveEncoder(driveEncoderPorts[0], driveEncoderPorts[1]),
//       m_turningEncoder(turningEncoderPorts[0], turningEncoderPorts[1]),
//       m_reverseDriveEncoder(driveEncoderReversed),
//       m_reverseTurningEncoder(turningEncoderReversed) {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
//   m_driveEncoder.SetDistancePerPulse(
//       ModuleConstants::kDriveEncoderDistancePerPulse);

//   // Set the distance (in this case, angle) per pulse for the turning encoder.
//   // This is the the angle through an entire rotation (2 * std::numbers::pi)
//   // divided by the encoder resolution.
//   m_turningEncoder.SetDistancePerPulse(
//       ModuleConstants::kTurningEncoderDistancePerPulse);

//   // Limit the PID Controller's input range between -pi and pi and set the input
//   // to be continuous.
//   m_turningPIDController.EnableContinuousInput(
//       units::radian_t{-std::numbers::pi}, units::radian_t{std::numbers::pi});
// }
SwerveModule::SwerveModule(int m_MotorController, rev::SparkMaxRelativeEncoder::Type m_EncoderType, int m_counts_per_rev, 
int m_MotorControllerTurning, 
 bool driveEncoderReversed,
 int TurningEncoderNumber,
 bool turningEncoderReversed
)

{
         this->m_EncoderType = m_EncoderType;
         this->m_counts_per_rev = m_counts_per_rev;
         samDriveMotor = new rev::CANSparkMax(m_MotorController, rev::CANSparkMax::MotorType::kBrushless);
         samTurningMotor = new rev::CANSparkMax(m_MotorControllerTurning, rev::CANSparkMax::MotorType::kBrushless);
        //  samDriveEncoder = new rev::CANEncoder(*samDriveMotor, m_EncoderType, m_counts_per_rev);
         samDriveEncoder = new rev::SparkMaxRelativeEncoder(samTurningMotor->GetEncoder(m_EncoderType, m_counts_per_rev));
        //  samTurningEncoder = new rev::CANEncoder(*samTurningMotor, m_EncoderTypeTurning, m_counts_per_revTurning);
         samTurningEncoder = new ctre::phoenix::sensors::CANCoder(TurningEncoderNumber);	

         m_reverseDriveEncoder = driveEncoderReversed;
         m_reverseTurningEncoder = turningEncoderReversed;
         samTurningEncoder->ConfigSensorDirection(m_reverseTurningEncoder);
        //Can't independently invert drive endcoder from drive motor.

  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  samDriveEncoder->SetPositionConversionFactor(
      ModuleConstants::kDriveEncoderDistancePerPulse);

  // Set the distance (in this case, angle, radians) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * wpi::numbers::pi)
  // divided by the encoder resolution.
  samTurningEncoder->ConfigFeedbackCoefficient(
  0.00153980788, "Radians", ctre::phoenix::sensors::SensorTimeBase());
  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                               units::radian_t(wpi::numbers::pi));
  m_drivePIDController.SetP(
    frc::SmartDashboard::PutNumber("Enter P Value" + std::to_string(samDriveMotor->GetDeviceId()),
     ModuleConstants::kPModuleDriveController));
  m_turningPIDController.SetP(
    frc::SmartDashboard::PutNumber("Enter P Value for Turn" + std::to_string(samTurningMotor->GetDeviceId()), 
    ModuleConstants::kPModuleTurningController));

  frc::SmartDashboard::PutNumber("Wheel Offset " + std::to_string(samTurningMotor->GetDeviceId()), ModuleConstants::wheelOffset);

}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveEncoder.GetRate()},
          units::radian_t{m_turningEncoder.GetDistance()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {units::meter_t{m_driveEncoder.GetDistance()},
          units::radian_t{m_turningEncoder.GetDistance()}};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t{m_turningEncoder.GetDistance()});

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetRate(), state.speed.value());

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t{m_turningEncoder.GetDistance()}, state.angle.Radians());

  // Set the motor outputs.
  m_driveMotor.Set(driveOutput);
  m_turningMotor.Set(turnOutput);
}

void SwerveModule::ResetEncoders() {
  m_driveEncoder.Reset();
  m_turningEncoder.Reset();
}
