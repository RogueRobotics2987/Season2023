// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/BeginCompressor.h"

BeginCompressor::BeginCompressor(CompressorObject &compressor) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_compressor = &compressor;
  AddRequirements({m_compressor});
}

// Called when the command is initially scheduled.
void BeginCompressor::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void BeginCompressor::Execute() {
  m_compressor->StartCompressor();
}

// Called once the command ends or is interrupted.
void BeginCompressor::End(bool interrupted) {}

// Returns true when the command should end.
bool BeginCompressor::IsFinished() {
  return false;
}
