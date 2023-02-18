// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/JoystickButton.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  //m_lights.SetDefaultCommand(LightsCmd(m_lights, m_xbox, m_newXbox));

  // Configure the button bindings
  ConfigureBindings();
  
}
void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
  frc2::JoystickButton(&m_xbox, 1).OnTrue(m_lights.ConeDesired());
  frc2::JoystickButton(&m_xbox, 2).OnTrue(m_lights.CubeDesired());
  frc2::JoystickButton(&m_xbox, 3).OnTrue(m_lights.RedColor());
  frc2::JoystickButton(&m_xbox, 4).OnTrue(m_lights.BlueColor());
  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`)
 
}


