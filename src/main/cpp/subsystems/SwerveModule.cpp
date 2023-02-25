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
){
  this->m_EncoderType = m_EncoderType;
  this->m_counts_per_rev = m_counts_per_rev;
  m_driveMotor = new rev::CANSparkMax(m_MotorController, rev::CANSparkMax::MotorType::kBrushless);
  m_driveMotor->SetOpenLoopRampRate(1);
  m_turningMotor = new rev::CANSparkMax(m_MotorControllerTurning, rev::CANSparkMax::MotorType::kBrushless);
  m_turningMotor->SetInverted(true);
  m_turningMotor->SetOpenLoopRampRate(0);
  //  samDriveEncoder = new rev::CANEncoder(*samDriveMotor, m_EncoderType, m_counts_per_rev);
  m_driveEncoder = new rev::SparkMaxRelativeEncoder(m_driveMotor->GetEncoder(m_EncoderType, m_counts_per_rev));
  //  samTurningEncoder = new rev::CANEncoder(*samTurningMotor, m_EncoderTypeTurning, m_counts_per_revTurning);
  m_turningEncoder = new ctre::phoenix::sensors::CANCoder(TurningEncoderNumber);	

  m_reverseDriveEncoder = driveEncoderReversed;
  m_reverseTurningEncoder = turningEncoderReversed;
  m_turningEncoder->ConfigSensorDirection(m_reverseTurningEncoder);
  //Can't independently invert drive endcoder from drive motor.

  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_driveEncoder->SetPositionConversionFactor(
    ModuleConstants::kDriveEncoderDistancePerPulse);
  m_driveEncoder->SetVelocityConversionFactor(
    ModuleConstants::kDriveEncoderDistancePerPulse / 60.0); //Converting RPM to Meters per second

  // Set the distance (in this case, angle, radians) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * wpi::numbers::pi)
  // divided by the encoder resolution.
  m_turningEncoder->ConfigFeedbackCoefficient(0.001534,"Radians", 
                                              ctre::phoenix::sensors::SensorTimeBase());
  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(units::radian_t(-std::numbers::pi),
                                               units::radian_t(std::numbers::pi));
  m_drivePIDController.SetP(
    frc::SmartDashboard::PutNumber("Enter P Value for Drive" + std::to_string(m_driveMotor->GetDeviceId()),
    ModuleConstants::kPModuleDriveController));
  m_turningPIDController.SetP(
    frc::SmartDashboard::PutNumber("Enter P Value for Turn" + std::to_string(m_turningMotor->GetDeviceId()), 
    ModuleConstants::kPModuleTurningController));
  m_turningPIDController.SetTolerance(units::radian_t(10));
  frc::SmartDashboard::PutNumber("KFF Input " + std::to_string(m_driveMotor->GetDeviceId()), ModuleConstants::kFFModuleDriveController);

  frc::SmartDashboard::PutNumber("Wheel Offset " + std::to_string(m_turningMotor->GetDeviceId()), ModuleConstants::wheelOffset);

}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveEncoder->GetVelocity()},
          units::radian_t{m_turningEncoder->GetPosition() + ModuleConstants::wheelOffset}};
          // units::radian_t{m_turningEncoder->GetPosition()}};
          //Subtracts ModuleConstants::wheelOffset becuse we add it in setDesired state
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {units::meter_t{m_driveEncoder->GetPosition()},
          units::radian_t{m_turningEncoder->GetPosition() + ModuleConstants::wheelOffset}};
          // units::radian_t{m_turningEncoder->GetPosition()}};
          //Subtracts ModuleConstants::wheelOffset becuse we add it in setDesired state
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
  double m_wheelOffset = frc::SmartDashboard::GetNumber("Wheel Offset " 
    + std::to_string(m_turningMotor->GetDeviceId()), ModuleConstants::wheelOffset);
  // Optimize the reference state to avoid spinning further than 90 degrees

  m_drivePIDController.SetP(frc::SmartDashboard::GetNumber("Enter P Value for Drive" + std::to_string(m_driveMotor->GetDeviceId()), 1E-5));
  auto driveOutput = m_drivePIDController.Calculate(
     m_driveEncoder->GetVelocity(), referenceState.speed.to<double>());

  double KFFInput = frc::SmartDashboard::GetNumber("KFF Input " + std::to_string(m_driveMotor->GetDeviceId()), ModuleConstants::kFFModuleDriveController);

  driveOutput = driveOutput + referenceState.speed.to<double>() * KFFInput;

  m_turningPIDController.SetP(
      frc::SmartDashboard::GetNumber("Enter P Value for Turn" + std::to_string(m_turningMotor->GetDeviceId()), 1E-5));
  
  auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t( m_turningEncoder->GetPosition() /* * 78.73*/ + m_wheelOffset), referenceState.angle.Radians());

  // double errorTest = m_turningEncoder->GetPosition() - referenceState.angle.Radians().value();

  // while(fabs(errorTest)>M_PI){
  //   if(errorTest>0){
  //     errorTest=errorTest-M_PI;
  //   }else{
  //     errorTest=errorTest+M_PI;
  //   }
  // }
  
  // if(fabs(errorTest)<0.06){
  //   turnOutput=0.0;
  // }
  
  // commented out to test, 2/17
  // frc::SmartDashboard::PutNumber("Drive Output " + std::to_string(m_driveMotor->GetDeviceId()), driveOutput);

  // frc::SmartDashboard::PutNumber("Get Velocity output" + std::to_string(m_driveMotor->GetDeviceId()), 
  //                               m_driveEncoder->GetVelocity());
  // frc::SmartDashboard::PutNumber("Velocity Command " + std::to_string(m_driveMotor->GetDeviceId()),
  //                               referenceState.speed.to<double>());
  // frc::SmartDashboard::PutNumber("Get Drive Positon" + std::to_string(m_driveMotor->GetDeviceId()), 
  //                               m_driveEncoder->GetPosition());
  // frc::SmartDashboard::PutNumber("get rotation Position " + std::to_string(m_turningMotor->GetDeviceId()), 
  //                                m_turningEncoder->GetPosition() *180/M_PI);
  // frc::SmartDashboard::PutNumber("Motor Set Position - " + std::to_string(m_turningMotor->GetDeviceId()),
  //                                double(referenceState.angle.Radians()));
  // frc::SmartDashboard::PutNumber("Turning Motor output" + std::to_string(m_turningMotor->GetDeviceId()), turnOutput);
  // //frc::SmartDashboard::PutNumber("Turning Motor error" + std::to_string(m_turningMotor->GetDeviceId()), fabs(fmod(errorTest,M_PI)));

  // frc::SmartDashboard::PutBoolean("Turning motor at setpoint",m_turningPIDController.AtSetpoint());

  // const auto state = frc::SwerveModuleState::Optimize(
  //     referenceState, units::radian_t{m_turningEncoder.GetDistance()});

  // // Calculate the drive output from the drive PID controller.
  // const auto driveOutput = m_drivePIDController.Calculate(
  //     m_driveEncoder.GetRate(), state.speed.value());

  // // Calculate the turning motor output from the turning PID controller.
  // auto turnOutput = m_turningPIDController.Calculate(
  //     units::radian_t{m_turningEncoder.GetDistance()}, state.angle.Radians());

  // Set the motor outputs.
  m_driveMotor->Set(driveOutput);
  m_turningMotor->Set(turnOutput);
}

void SwerveModule::ResetEncoders() {
  // m_driveEncoder->Reset();
  // m_turningEncoder->Reset();
}

void SwerveModule::ConfigMotorControllers(){
  m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_turningMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

  frc::ProfiledPIDController<units::radians> SwerveModule::GetTurnPID(){
  return m_turningPIDController;
}


SwerveModule::~SwerveModule(){
  delete m_driveMotor;
  delete m_turningMotor;
  delete m_driveEncoder;
  delete m_turningEncoder;
}

void SwerveModule::Periodic() {

// {double m_wheelOffset = frc::SmartDashboard::GetNumber("Wheel Offset " 
//     + std::to_string(m_turningMotor->GetDeviceId()), ModuleConstants::wheelOffset);
//   // Optimize the reference state to avoid spinning further than 90 degrees

//   m_drivePIDController.SetP(frc::SmartDashboard::GetNumber("Enter P Value" + std::to_string(m_driveMotor->GetDeviceId()), 1E-5));
//   const auto driveOutput = m_drivePIDController.Calculate(
//      (m_driveEncoder->GetVelocity(), referenceState.speed.to<double>()) / 10);

//   m_turningPIDController.SetP(
//       frc::SmartDashboard::GetNumber("Enter P Value for Turn" + std::to_string(m_turningMotor->GetDeviceId()), 1E-5));
  
//   auto turnOutput = m_turningPIDController.Calculate(
//       units::radian_t( m_turningEncoder->GetPosition() /* * 78.73*/ + m_wheelOffset), referenceState.angle.Radians());
  
//   frc::SmartDashboard::PutNumber(std::to_string(m_driveMotor->GetDeviceId()), driveOutput);

//   frc::SmartDashboard::PutNumber("Get Velocity output" + std::to_string(m_driveMotor->GetDeviceId()), 
//                                 m_driveEncoder->GetVelocity() / 10);
//   frc::SmartDashboard::PutNumber("Get Drive Positon" + std::to_string(m_driveMotor->GetDeviceId()), 
//                                 m_driveEncoder->GetPosition());
//   frc::SmartDashboard::PutNumber("get rotation Position" + std::to_string(m_turningMotor->GetDeviceId()), 
//                                  m_turningEncoder->GetPosition() + m_wheelOffset /* * 78.73*/);
//   frc::SmartDashboard::PutNumber("Motor Set Position - " + std::to_string(m_turningMotor->GetDeviceId()),
//                                  double(referenceState.angle.Radians()) /* * 78.73*/);
//   frc::SmartDashboard::PutNumber(std::to_string(m_turningMotor->GetDeviceId()), turnOutput);

}