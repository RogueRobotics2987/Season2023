// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include "rev/SparkMaxRelativeEncoder.h"

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

// namespace DriveConstants {
// constexpr int kFrontLeftDriveMotorPort = 0;
// constexpr int kRearLeftDriveMotorPort = 2;
// constexpr int kFrontRightDriveMotorPort = 4;
// constexpr int kRearRightDriveMotorPort = 6;

// constexpr int kFrontLeftTurningMotorPort = 1;
// constexpr int kRearLeftTurningMotorPort = 3;
// constexpr int kFrontRightTurningMotorPort = 5;
// constexpr int kRearRightTurningMotorPort = 7;

// constexpr int kFrontLeftTurningEncoderPorts[2]{0, 1};
// constexpr int kRearLeftTurningEncoderPorts[2]{2, 3};
// constexpr int kFrontRightTurningEncoderPorts[2]{4, 5};
// constexpr int kRearRightTurningEncoderPorts[2]{6, 7};

// constexpr bool kFrontLeftTurningEncoderReversed = false;
// constexpr bool kRearLeftTurningEncoderReversed = true;
// constexpr bool kFrontRightTurningEncoderReversed = false;
// constexpr bool kRearRightTurningEncoderReversed = true;

// constexpr int kFrontLeftDriveEncoderPorts[2]{8, 9};
// constexpr int kRearLeftDriveEncoderPorts[2]{10, 11};
// constexpr int kFrontRightDriveEncoderPorts[2]{12, 13};
// constexpr int kRearRightDriveEncoderPorts[2]{14, 15};

// constexpr bool kFrontLeftDriveEncoderReversed = false;
// constexpr bool kRearLeftDriveEncoderReversed = true;
// constexpr bool kFrontRightDriveEncoderReversed = false;
// constexpr bool kRearRightDriveEncoderReversed = true;

// // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// // These characterization values MUST be determined either experimentally or
// // theoretically for *your* robot's drive. The SysId tool provides a convenient
// // method for obtaining these values for your robot.
// constexpr auto ks = 1_V;
// constexpr auto kv = 0.8 * 1_V * 1_s / 1_m;
// constexpr auto ka = 0.15 * 1_V * 1_s * 1_s / 1_m;

// // Example value only - as above, this must be tuned for your drive!
// constexpr double kPFrontLeftVel = 0.5;
// constexpr double kPRearLeftVel = 0.5;
// constexpr double kPFrontRightVel = 0.5;
// constexpr double kPRearRightVel = 0.5;
// }  // namespace DriveConstants

namespace DriveConstants {
constexpr int kFrontLeftDriveMotorPort = 13;
constexpr int kRearLeftDriveMotorPort = 7;
constexpr int kFrontRightDriveMotorPort = 3;
constexpr int kRearRightDriveMotorPort = 5;

constexpr int kFrontLeftTurningMotorPort = 2;
constexpr int kRearLeftTurningMotorPort = 8;
constexpr int kFrontRightTurningMotorPort = 4;
constexpr int kRearRightTurningMotorPort = 6;

constexpr int kFrontLeftTurningEncoderNumber = 9;
constexpr int kRearLeftTurningEncoderNumber = 12;
constexpr int kFrontRightTurningEncoderNumber = 10;
constexpr int kRearRightTurningEncoderNumber = 11;

// constexpr int Actuator = 40;

// constexpr int kFrontLeftTurningEncoderPorts[2]{0, 1};
// constexpr int kRearLeftTurningEncoderPorts[2]{2, 3};
// constexpr int kFrontRightTurningEncoderPorts[2]{4, 5};
// constexpr int kRearRightTurningEncoderPorts[2]{5, 6};

constexpr bool kFrontLeftTurningEncoderReversed = false;
constexpr bool kRearLeftTurningEncoderReversed = false;
constexpr bool kFrontRightTurningEncoderReversed = false;
constexpr bool kRearRightTurningEncoderReversed = false;

// constexpr int kFrontLeftDriveEncoderPorts[2]{0, 1};
// constexpr int kRearLeftDriveEncoderPorts[2]{2, 3};
// constexpr int kFrontRightDriveEncoderPorts[2]{4, 5};
// constexpr int kRearRightDriveEncoderPorts[2]{5, 6};

constexpr bool kFrontLeftDriveEncoderReversed = false;
constexpr bool kRearLeftDriveEncoderReversed = true;
constexpr bool kFrontRightDriveEncoderReversed = false;
constexpr bool kRearRightDriveEncoderReversed = true;

constexpr int kFrontLeftDriveCPR = 42;
constexpr int kRearLeftDriveCPR = 42;
constexpr int kFrontRightDriveCPR = 42;
constexpr int kRearRightDriveCPR = 42;

constexpr int kFrontLeftTurningCPR = 1;
constexpr int kRearLeftTurningCPR = 1;
constexpr int kFrontRightTurningCPR = 1;
constexpr int kRearRightTurningCPR = 1;

// constexpr rev::CANEncoder::EncoderType m_EncoderType = rev::CANEncoder::EncoderType::kHallSensor;
constexpr rev::SparkMaxRelativeEncoder::Type m_EncoderType = rev::SparkMaxRelativeEncoder::Type::kHallSensor;

// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The RobotPy Characterization
// Toolsuite provides a convenient tool for obtaining these values for your
// robot.
constexpr auto ks = 1_V;
constexpr auto kv = 0.8 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.15 * 1_V * 1_s * 1_s / 1_m;

// Example value only - as above, this must be tuned for your drive!
constexpr double kPFrontLeftVel = 0.5;
constexpr double kPRearLeftVel = 0.5;
constexpr double kPFrontRightVel = 0.5;
constexpr double kPRearRightVel = 0.5;
}  // namespace DriveConstants


// namespace ModuleConstants {
// constexpr int kTurnEncoderCPR = 1024;
// constexpr double kWheelDiameterMeters = 0.15;
// constexpr double kDriveEncoderDistancePerPulse =
//     // Assumes the encoders are directly mounted on the wheel shafts
//     (kWheelDiameterMeters * std::numbers::pi) /
//     static_cast<double>(kTurnEncoderCPR);

// constexpr double kTurningEncoderDistancePerPulse =
//     // Assumes the encoders are directly mounted on the wheel shafts
//     (std::numbers::pi * 2) / static_cast<double>(kTurnEncoderCPR);

// constexpr double kPModuleTurningController = 1;
// constexpr double kPModuleDriveController = 1;
// }  // namespace ModuleConstants

namespace ModuleConstants {
constexpr double wheelOffset = 0;
constexpr double gearRatio = 8.16; //we measured 8.91
constexpr int kTurnEncoderCPR = 1;
constexpr int  kDriveEncoder = 42; //TODO -KBK Add Term?
constexpr double kWheelDiameterMeters = 0.0977; // 0.0762

constexpr double kDriveEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    //(kWheelDiameterMeters * std::numbers::pi) / static_cast<double>(kTurnEncoderCPR) / gearRatio;
    (kWheelDiameterMeters * std::numbers::pi) / static_cast<double>(kDriveEncoder) / gearRatio; //TODO -KBK Add Term?
    //TODO - KBK - The Drive Encoders and Turn Encocers have different Counts Per Rev, We likely need to use another constant in this equation.

constexpr double kTurningEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (std::numbers::pi * 2) / static_cast<double>(kTurnEncoderCPR);

constexpr double kPModuleTurningController = 0.5;
constexpr double kPModuleDriveController = 8;
// TODO Lower Value of P to 0.0001,  Change Value of p Till its the Highest Without Osilation, 
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

//

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants
