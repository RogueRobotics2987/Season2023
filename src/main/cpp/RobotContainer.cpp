// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>



using namespace DriveConstants;
using namespace pathplanner;


RobotContainer::RobotContainer() {

    ConfigMotorControllers();
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  m_elevator.SetDefaultCommand(ElevatorCmd(m_elevator, m_xbox, m_stick1));
  m_compressor.SetDefaultCommand(BeginCompressor(m_compressor));



  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      //   std::cout << "sea out in robot container" << std::endl;
      // commented out to test, 2/17
      // frc::SmartDashboard::PutNumber("Xbox left  X axis", m_xbox.GetLeftX());
      // frc::SmartDashboard::PutNumber("Xbox Left y axis", m_xbox.GetLeftY());
      // frc::SmartDashboard::PutNumber("Xbox Right X axis", m_xbox.GetRightX());
      
      bool noJoystick = false;
      bool noJoystickX = false;
      bool noJoystickY = false;
      bool noJoystickRot = false;
      double safeX = m_newXbox.GetLeftX();
      if(fabs(safeX)<0.1) {
          safeX=0;
          noJoystickX = true;
      }
      double safeY =  m_newXbox.GetLeftY();
      if(fabs(safeY)<0.1) { 
          safeY=0;
          noJoystickY = true;
      }
      double safeRot = m_newXbox.GetRightX();
      if(fabs(safeRot)<0.1) {
          safeRot=0;
          noJoystickRot = true;
      }
      noJoystick = noJoystickX && noJoystickY && noJoystickRot;

      frc::SmartDashboard::PutNumber("noJoystick val ", noJoystick);
      
      // std::cout << "Sam Debug" << safeX << "," << safeY << "," << safeRot << std::endl;
      bool fieldOrientated = frc::SmartDashboard::GetBoolean("Field orientated control", false); 
      m_drive.Drive(units::meters_per_second_t(
                        -safeY * AutoConstants::kMaxSpeed),
                    units::meters_per_second_t(
                        -safeX * AutoConstants::kMaxSpeed),
                    units::radians_per_second_t(
                        -safeRot * PI),
                    fieldOrientated,
                    noJoystick);
      // m_drive.Drive(units::meters_per_second_t(0),
      // units::meters_per_second_t(1),
      // units::radians_per_second_t(0),
      // false);
    }, {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_stick1, 7).OnTrue(m_drive.SetDriveSlow(true));
  frc2::JoystickButton(&m_stick1, 7).OnFalse(m_drive.SetDriveSlow(false));
  frc2::JoystickButton(&m_stick1, 1).OnTrue(m_drive.ButtonZeroHeading());

  frc2::JoystickButton(&m_stick1, 2).OnTrue(m_drive.ConfigOdometry());


  frc2::JoystickButton(&m_xbox, 5).OnTrue(m_elevator.ClawOpenCommand());
  frc2::JoystickButton(&m_xbox, 6).OnFalse(m_elevator.ClawCloseCommand());
  frc2::JoystickButton(&m_newXbox, 3).OnTrue(m_elevator.ClawOpenCommand()); //Button X
  frc2::JoystickButton(&m_newXbox, 1).OnFalse(m_elevator.ClawCloseCommand()); //Button A

  //frc2::JoystickButton(&m_stick1, 14).OnTrue(m_elevator.SetPlaceHighState())
  //frc2::JoystickButton(&m_stick1, 15).OnTrue(m_elevator.SetPlaceMidState());
  //frc2::JoystickButton(&m_stick1, 16).OnTrue(m_elevator.SetPlaceLowState());
  frc2::JoystickButton(&m_stick1, 5).OnTrue(m_elevator.SetManualElevatorState());//need to change

  frc2::JoystickButton(&m_newXbox, 7).OnTrue(m_drive.FieldOrientatedTrue());
  frc2::JoystickButton(&m_newXbox, 8).OnTrue(m_drive.FieldOrientatedFalse());
  
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction

      //x moves forward backward

      //y moves left right

      frc::Pose2d{0_m, 0_m, 0_deg},
      {frc::Translation2d{1_m, 0_m} , frc::Translation2d{1_m, 1_m}, frc::Translation2d{0_m, 1_m}},
      frc::Pose2d{0_m, 0_m, 0_deg},

      // frc::Pose2d{0_m, 0_m, 0_deg},
      // {frc::Translation2d{1_m, 1_m} , frc::Translation2d{2_m, 0_m}, frc::Translation2d{3_m, -1_m}},
      // frc::Pose2d{4_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() {return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

       frc2::PIDController{AutoConstants::kPXController, 0, 0},
      frc2::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); }, {}));
}

  void RobotContainer::ZeroHeading(){
    m_drive.ZeroHeading();
}

  void RobotContainer::ConfigMotorControllers(){
    m_drive.ConfigMotorControllers();
  }

  void RobotContainer::ResetOdometry(){
    m_drive.ResetOdometry(frc::Pose2d{5_m, 5_m, 0_deg});
  }


    //Commented out for merge purposes 

  // frc2::Command* RobotContainer::GetPathCommand(){
  //   // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
  //   PathPlannerTrajectory examplePath = PathPlanner::loadPath("New Path", PathConstraints(1.5_mps, 0.5_mps_sq));

  //  // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
  // // for every path in the group
  // std::vector<PathPlannerTrajectory> pathGroup = PathPlanner::loadPathGroup("FullAuto", {PathConstraints(4_mps, 3_mps_sq)});


  // // This is just an example event map. It would be better to have a constant, global event map
  // // in your code that will be used by all path following commands/autobuilders.
  // std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;
  // eventMap.emplace("marker1", std::make_shared<frc2::PrintCommand>("Passed Marker 1"));
  // // eventMap.emplace("intakeDown", std::make_shared<IntakeDown>());

  // // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this could be in RobotContainer along with your subsystems

  // SwerveAutoBuilder autoBuilder(
  //     [this]() { return swerveSubsystem.getPose(); }, // Function to supply current robot pose
  //     [this](auto initPose) { swerveSubsystem.resetPose(initPose); }, // Function used to reset odometry at the beginning of auto
  //     PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
  //     PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
  //     [this](auto speeds) { swerveSubsystem.driveFieldRelative(speeds); }, // Output function that accepts field relative ChassisSpeeds
  //     eventMap, // Our event map
  //     { &swerveSubsystem }, // Drive requirements, usually just a single drive subsystem
  //     true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  // );

  // CommandPtr fullAuto = autoBuilder.fullAuto(pathGroup);
    // }
