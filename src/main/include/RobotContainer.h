// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h> //TODO this libaray list is bloated
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/interfaces/Gyro.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <cameraserver/CameraServer.h>
#include <iostream>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <utility>

#include "Constants.h"
#include "commands/BeginCompressor.h"
#include "commands/ElevatorCmd.h"
#include "commands/AutoBalance.h"
#include "commands/LightsCmd.h"
#include "subsystems/Lights.h"
#include "subsystems/SwerveModule.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Elevator.h"
#include "commands/PlaceAutoCmd.h"
#include "commands/TimerCMD.h"
#include "commands/TimedBalanceCmd.h"
#include "commands/InPlaceRotationCmd.h"

using namespace DriveConstants;
using namespace pathplanner;
using namespace frc2;



/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();


  // void ConfigureBindings();
  frc2::Command* GetAutonomousCommand();


  frc2::CommandPtr DriveCrgStnBlue1(DriveSubsystem &m_drive);
  frc2::CommandPtr DriveCrgStnBlue2(DriveSubsystem &m_drive);

  frc2::CommandPtr DriveCrgStnRed1(DriveSubsystem &m_drive);
  frc2::CommandPtr DriveCrgStnRed2(DriveSubsystem &m_drive);

  frc2::CommandPtr PlaceDriveCrgStnRed1(DriveSubsystem &m_drive);
  frc2::CommandPtr PlaceDriveCrgStnBlue1(DriveSubsystem &m_drive);

  frc2::CommandPtr ConeBalanceBlue(DriveSubsystem &m_drive);
  frc2::CommandPtr ConeBalanceRed(DriveSubsystem &m_drive);

  frc2::CommandPtr RedLeave1(DriveSubsystem &m_drive);
  frc2::CommandPtr RedLeave2(DriveSubsystem &m_drive);
  frc2::CommandPtr BlueLeave1(DriveSubsystem &m_drive);
  frc2::CommandPtr BlueLeave2(DriveSubsystem &m_drive);

  frc2::CommandPtr Red2Place1(DriveSubsystem &m_drive);
  frc2::CommandPtr Red2Place2(DriveSubsystem &m_drive);
  frc2::CommandPtr Red2Place3(DriveSubsystem &m_drive);

  frc2::CommandPtr Blue2Place1(DriveSubsystem &m_drive);
  frc2::CommandPtr Blue2Place2(DriveSubsystem &m_drive);

  frc2::CommandPtr Blue2Place1Spin(DriveSubsystem &m_drive);
  frc2::CommandPtr Blue2Place2Spin(DriveSubsystem &m_drive); 
  frc2::CommandPtr Blue2Place3Spin(DriveSubsystem &m_drive);

  frc2::CommandPtr BlueBump1(DriveSubsystem &m_drive);
  frc2::CommandPtr BlueBump2(DriveSubsystem &m_drive); 
  frc2::CommandPtr BlueBump3(DriveSubsystem &m_drive);  

  frc2::CommandPtr RedBump1(DriveSubsystem &m_drive);
  frc2::CommandPtr RedBump2(DriveSubsystem &m_drive); 
  frc2::CommandPtr RedBump3(DriveSubsystem &m_drive); 

  frc2::CommandPtr Red2Place1Spin(DriveSubsystem &m_drive);
  frc2::CommandPtr Red2Place2Spin(DriveSubsystem &m_drive); 
  frc2::CommandPtr Red2Place3Spin(DriveSubsystem &m_drive);
  frc2::CommandPtr Red2Place4Spin(DriveSubsystem &m_drive); 

  frc2::CommandPtr BlueMid(DriveSubsystem &m_drive);
  frc2::CommandPtr RedMid(DriveSubsystem &m_drive);  

  frc2::CommandPtr Forward1(DriveSubsystem &m_drive);
  frc2::CommandPtr Forward2(DriveSubsystem &m_drive);

  frc2::CommandPtr Slide1(DriveSubsystem &m_drive);
  frc2::CommandPtr Slide2(DriveSubsystem &m_drive);

  frc2::CommandPtr Spin1(DriveSubsystem &m_drive);

  frc2::CommandPtr Rotate180(DriveSubsystem &m_drive);

  frc2::CommandPtr Forwards45(DriveSubsystem &m_drive);
  frc2::CommandPtr Backwards45(DriveSubsystem &m_drive);
  
  frc2::CommandPtr BlueOverStn(DriveSubsystem &m_drive);
  frc2::CommandPtr RedOverStn(DriveSubsystem &m_drive);


  frc::DriverStation::Alliance AllianceColor = frc::DriverStation::GetAlliance();

  void SetRanAuto(bool ranAuto);

  double GetHeading();

  double GetOdometry();
  
  void ZeroHeading();

  void ConfigMotorControllers();

  void ResetOdometry();

  float Deadzone(float x, bool rotation);

  frc2::CommandPtr GetPathCommand();

 private:
  // The driver's controller
  //frc::Joystick m_stick1{2};
  frc::XboxController m_xbox{0};
  frc::XboxController m_newXbox{1};
  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  Elevator m_elevator;
  CompressorObject m_compressor;
  Lights m_lights;

  //frc2::Command *AutoCmd;

  frc2::Command* AutoCmd = new AutoBalance(m_drive);

  frc2::Command* Spin180Cmd = new InPlaceRotationCmd(179, m_drive);
  frc2::Command* Spin180Cmd2 = new InPlaceRotationCmd(179, m_drive);

  frc2::Command* TimedBalLeftCmd = new TimedBalanceCmd("Left", m_drive);
  frc2::Command* TimedBalRightCmd = new TimedBalanceCmd("Right", m_drive);

  frc2::Command* PlaceHighCmd = new PlaceAutoCmd(m_elevator, 106, -30, 160); // 104 for height, was 260 for tilt before new gear ratio
  frc2::Command* PlaceMidCmd = new PlaceAutoCmd(m_elevator, 53, -63.6, 87); // 104 for height
  frc2::Command* PlaceLowCmd = new PlaceAutoCmd(m_elevator, 106, -45, 160); // 104 for height
  frc2::Command* PickupCmd = new PlaceAutoCmd(m_elevator, 70.4, -90, 54);//70.4, -90,54
  frc2::Command* RetractCmdAuto = new PlaceAutoCmd(m_elevator, 5, 0, 0);
  frc2::Command* RetractPickupCmdAuto = new PlaceAutoCmd(m_elevator, 5, -10, 90);
  frc2::Command* RetractPickupCmdAuto2 = new PlaceAutoCmd(m_elevator, 29, -10, 90);
  frc2::Command* RetractCmd = new PlaceAutoCmd(m_elevator, 5, 0, 0);
  frc2::Command* PickupTipCmd = new PlaceAutoCmd(m_elevator, 29, -171.5, 132);  //30, -171.5, 242 //was 30 fro height
  frc2::Command* PickupFlatAuto = new PlaceAutoCmd(m_elevator, 29, -171.5, 132);//69, -110, 0
  frc2::Command* PickupWithBumpersIntoSubstation = new PlaceAutoCmd(m_elevator, 70, -121, 0);//69, -110, 0 //was 61.5, -103, 0
  frc2::Command* PickupGroundStandingCone = new PlaceAutoCmd(m_elevator, 5, -88, 160);

  frc2::ParallelRaceGroup* PlaceHighRace = new ParallelRaceGroup(TimerCMD(0.5), PlaceAutoCmd(m_elevator, 104, -60, 160));
  frc2::ParallelRaceGroup* PlaceHighRace2 = new ParallelRaceGroup(TimerCMD(0.75), PlaceAutoCmd(m_elevator, 104, -30, 160),
                    frc2::RunCommand([this](){m_drive.Drive(units::meters_per_second_t(0),
                      units::meters_per_second_t(0),
                      units::radians_per_second_t(0),false,false);}, {&m_drive}
                      ));
  frc2::ParallelRaceGroup* PlaceHighRace3 = new ParallelRaceGroup(TimerCMD(1), PlaceAutoCmd(m_elevator, 104, -60, 160),
                    frc2::RunCommand([this](){m_drive.Drive(units::meters_per_second_t(0),
                      units::meters_per_second_t(0),
                      units::radians_per_second_t(0),false,false);}, {&m_drive}
                      ));
  frc2::ParallelRaceGroup* StandStillRace1 = new ParallelRaceGroup(TimerCMD(0.1),
                    frc2::RunCommand([this](){m_drive.Drive(units::meters_per_second_t(0),
                      units::meters_per_second_t(0),
                      units::radians_per_second_t(0),false,false);}, {&m_drive}
                      ));
  frc2::ParallelRaceGroup* PickupRace1 = new ParallelRaceGroup(PlaceAutoCmd(m_elevator, 29, -10, 90),
                    frc2::RunCommand([this](){m_drive.Drive(units::meters_per_second_t(0),
                      units::meters_per_second_t(0),
                      units::radians_per_second_t(0),false,false);}, {&m_drive}
                      ));                    

  frc2::ParallelRaceGroup* initialPlaceRace = new ParallelRaceGroup(TimerCMD(2), PlaceAutoCmd(m_elevator, 104, -30, 160));




  // frc2::ParallelCommandGroup* RetractMove = new ParallelCommandGroup(RetractCmd, TimerCMD(0.5));
  // The chooser for the autonomous routines 
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();

  frc2::CommandPtr DriveCrgStnRed1cmd = DriveCrgStnRed1(m_drive);
  frc2::CommandPtr DriveCrgStnRed2cmd = DriveCrgStnRed2(m_drive);

  frc2::CommandPtr DriveCrgStnBlue1cmd = DriveCrgStnBlue1(m_drive);
  frc2::CommandPtr DriveCrgStnBlue2cmd = DriveCrgStnBlue2(m_drive);

  frc2::CommandPtr PlaceDriveCrgStnRed1cmd = PlaceDriveCrgStnRed1(m_drive);
  frc2::CommandPtr PlaceDriveCrgStnBlue1cmd = PlaceDriveCrgStnBlue1(m_drive); 

  frc2::CommandPtr Leave1Blue = BlueLeave1(m_drive); 
  frc2::CommandPtr Leave2Blue = BlueLeave2(m_drive); 

  frc2::CommandPtr Leave1Red = RedLeave1(m_drive); 
  frc2::CommandPtr Leave2Red = RedLeave2(m_drive); 

  frc2::CommandPtr Forward1Cmd = Forward1(m_drive); 
  frc2::CommandPtr Forward2Cmd = Forward2(m_drive); 

  frc2::CommandPtr Slide1Cmd = Slide1(m_drive); 
  frc2::CommandPtr Slide2Cmd = Slide2(m_drive); 

  frc2::CommandPtr Spin1Cmd = Spin1(m_drive); 

  frc2::CommandPtr Forwards45Cmd = Forwards45(m_drive); 
  frc2::CommandPtr Backwards45Cmd = Backwards45(m_drive); 

  frc2::CommandPtr ConeBalanceBlueCmd = ConeBalanceBlue(m_drive); 
  frc2::CommandPtr ConeBalanceRedCmd = ConeBalanceRed(m_drive);  

  frc2::CommandPtr Red2Place1Cmd = Red2Place1(m_drive);  
  frc2::CommandPtr Red2Place2Cmd = Red2Place2(m_drive);  
  frc2::CommandPtr Red2Place3Cmd = Red2Place3(m_drive);  

  frc2::CommandPtr Blue2Place1Cmd = Blue2Place1(m_drive);  
  frc2::CommandPtr Blue2Place2Cmd = Blue2Place2(m_drive);  

  frc2::CommandPtr Blue2Place1SpinCmd = Blue2Place1Spin(m_drive);  
  frc2::CommandPtr Blue2Place2SpinCmd = Blue2Place2Spin(m_drive);  
  frc2::CommandPtr Blue2Place3SpinCmd = Blue2Place3Spin(m_drive);  

  frc2::CommandPtr BlueBump1Cmd = BlueBump1(m_drive);  
  frc2::CommandPtr BlueBump2Cmd = BlueBump2(m_drive);  
  frc2::CommandPtr BlueBump3Cmd = BlueBump3(m_drive);  

  frc2::CommandPtr RedBump1Cmd = RedBump1(m_drive);  
  frc2::CommandPtr RedBump2Cmd = RedBump2(m_drive);  
  frc2::CommandPtr RedBump3Cmd = RedBump3(m_drive);      

  frc2::CommandPtr Red2Place1SpinCmd = Red2Place1Spin(m_drive);  
  frc2::CommandPtr Red2Place2SpinCmd = Red2Place2Spin(m_drive);
  frc2::CommandPtr Red2Place3SpinCmd = Red2Place3Spin(m_drive);  
  frc2::CommandPtr Red2Place4SpinCmd = Red2Place4Spin(m_drive);  
  
  frc2::CommandPtr BlueMidCmd = BlueMid(m_drive);  
  frc2::CommandPtr RedMidCmd = RedMid(m_drive);

  frc2::CommandPtr BlueOverStnCmd = BlueOverStn(m_drive);  
  frc2::CommandPtr RedOverStnCmd = RedOverStn(m_drive);  

  frc2::CommandPtr Rotate180Cmd = Rotate180(m_drive);


  frc2::CommandPtr AutoZeroHeading = m_drive.ZeroHeading();

  frc2::CommandPtr OpenClawCmd = m_elevator.ClawOpenCommand();
  frc2::CommandPtr CloseClawCmd = m_elevator.ClawCloseCommand();

  frc2::CommandPtr SetHighCmd = m_elevator.SetPlaceHighState();
  frc2::CommandPtr SetMidCmd = m_elevator.SetPlaceMidState();
  frc2::CommandPtr SetLowCmd = m_elevator.SetPlaceLowState();





};
