// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"


RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_elevator.SetDefaultCommand(ElevatorCmd(m_elevator, m_xbox, m_stick1));
  m_compressor.SetDefaultCommand(BeginCompressor(m_compressor));

  //m_elevator.SetDefaultCommand(ElevatorCmd());
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
  //frc2::JoystickButton(&m_stick1,3).OnTrue(m_elevator.MotorMoveCommand());
  //frc2::JoystickButton(&m_stick1,3).OnFalse(m_elevator.StopMoveCommand());
  frc2::JoystickButton(&m_stick1, 3).OnTrue(m_elevator.ClawOpenCommand());
  frc2::JoystickButton(&m_stick1, 4).OnFalse(m_elevator.ClawCloseCommand());

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
