// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Solenoid.h> 
#include <frc/DoubleSolenoid.h> 
#include "Constants.h"
#include <iostream>

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();
  //Elevator(Joystick& )

  frc2::CommandPtr ClawOpenCommand();
  frc2::CommandPtr ClawCloseCommand();
  frc2::CommandPtr SetPlaceHighState();
  frc2::CommandPtr SetPlaceMidState();
  frc2::CommandPtr SetPlaceLowState();
  frc2::CommandPtr SetManualElevatorState();
  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void ElevatorVert(double elevatorUp, double elevatorDown);
  void ElevatorTilt(double lean);
  void ElevatorArm(double armXboxVal);
  void Periodic() override;
  //void Open(int SolenoidNum);
  //void Close(int SolenoidNum);


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_vertElevatorMotorLeft = rev::CANSparkMax(10, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax m_vertElevatorMotorRight = rev::CANSparkMax(9, rev::CANSparkMax::MotorType::kBrushless);
  rev::SparkMaxLimitSwitch ls_vertElevator = m_vertElevatorMotorLeft.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);//forward limit switch
  rev::SparkMaxLimitSwitch ls_vertElevatorF = m_vertElevatorMotorLeft.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);//forward limit switch
  rev::SparkMaxRelativeEncoder re_vertElevator= m_vertElevatorMotorLeft.GetEncoder(); 

  enum ElevatorState_t {INIT, FIND_ZERO, MANUAL_MODE, PLACE_HIGH, PLACE_MID, PLACE_LOW}; 
  ElevatorState_t ElevatorState = FIND_ZERO;

  rev::CANSparkMax m_tiltElevatorMotor = rev::CANSparkMax(12, rev::CANSparkMax::MotorType::kBrushless);
  //thsi limit switch is supposed to be normally closed in Rev and noramlly open in code. Why? no idea
  rev::SparkMaxLimitSwitch ls_tiltElevator = m_tiltElevatorMotor.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen); //reverse limit switch
  rev::SparkMaxLimitSwitch ls_tiltElevatorF = m_tiltElevatorMotor.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen); //reverse limit switch
  rev::SparkMaxRelativeEncoder re_tiltElevator = m_tiltElevatorMotor.GetEncoder(); 

  rev::CANSparkMax m_armMotor = rev::CANSparkMax(11, rev::CANSparkMax::MotorType::kBrushless);//in brake mode and can only go -11 turns
  rev::SparkMaxLimitSwitch ls_arm = m_armMotor.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);//forward limit switch
  rev::SparkMaxLimitSwitch ls_armR = m_armMotor.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);//forward limit switch
  rev::SparkMaxRelativeEncoder re_arm = m_armMotor.GetEncoder(); 

  double verticalVal = 0.0;
  double tiltVal = 0.0;
  double armVal = 0.0;
  bool resetElevatorFinished = false;
  bool enableElevator = true; //when false, it turns off elevator for outreach events when kids have the robot
  
  //claw open and close on pneumatics
  frc::DoubleSolenoid clawSolenoid = frc::DoubleSolenoid(1, frc::PneumaticsModuleType::REVPH, 0, 7); 

};
