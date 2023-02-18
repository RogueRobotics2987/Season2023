// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>



using namespace DriveConstants;
using namespace pathplanner;


RobotContainer::RobotContainer() {

  m_chooser.SetDefaultOption("Simple Auto", m_simpleAuto.get()); //SetDefaultOption
  m_chooser.AddOption("Complex Auto", m_complexAuto.get());
  m_chooser.AddOption("Command Auto", m_complexAuto.get());



  frc::Shuffleboard::GetTab("Autonomous").Add(m_chooser);


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
          frc::SmartDashboard::PutNumber("Left Hand Y", m_stick1.GetLeftY());
          frc::SmartDashboard::PutNumber("Right Hand Y", m_stick1.GetRightY());
          frc::SmartDashboard::PutNumber("Left Hand X", m_stick1.GetLeftX());
        
        bool noJoystick = false;
        bool noJoystickX = false;
        bool noJoystickY = false;
        bool noJoystickRot = false;
        double safeX = m_stick1.GetLeftX();
        if(fabs(safeX)<0.3) {
            safeX=0;
            noJoystickX = true;
            }
        double safeY =  m_stick1.GetLeftY();
        if(fabs(safeY)<0.3) { 
            safeY=0;
            noJoystickY = true;
            }
        double safeRot = m_stick1.GetRightX();
        if(fabs(safeRot)<0.3) {
            safeRot=0;
            noJoystickRot = true;
            }
            noJoystick = noJoystickX && noJoystickY && noJoystickRot;

            frc::SmartDashboard::PutNumber("noJoystick val ", noJoystick);
            frc::SmartDashboard::PutNumber("SafeX Val: ", safeX);
            frc::SmartDashboard::PutNumber("SafeY Val: ", safeY);
            frc::SmartDashboard::PutNumber("SafeRot Val: ", safeRot);
        
        // std::cout << "Sam Debug" << safeX << "," << safeY << "," << safeRot << std::endl;
        
        m_drive.Drive(units::meters_per_second_t(
                         -safeY * AutoConstants::kMaxSpeed),
                      units::meters_per_second_t(
                         -safeX * AutoConstants::kMaxSpeed),
                      units::radians_per_second_t(
                         -safeRot * PI),
                      false,
                      noJoystick);
        // m_drive.Drive(units::meters_per_second_t(0),
        // units::meters_per_second_t(1),
        // units::radians_per_second_t(0),
        // false);
      },
      {&m_drive}));
}

// void RobotContainer::ConfigureButtonBindings() {
//     frc2::JoystickButton(&m_stick1, 7).OnTrue(m_drive.SetDriveSlow(true));
//     frc2::JoystickButton(&m_stick1, 7).OnFalse(m_drive.SetDriveSlow(false));
//     frc2::JoystickButton(&m_stick1, 1).OnTrue(m_drive.ButtonZeroHeading());
//     // frc2::JoystickButton(&m_stick1, 11).OnTrue(AutoCmd);
//     frc2::JoystickButton(&m_stick1, 11).OnTrue(AutoCmd);
//   m_drive.SetDefaultCommand(frc2::RunCommand(
//     [this] {
//       //   std::cout << "sea out in robot container" << std::endl;
//       frc::SmartDashboard::PutNumber("Left Hand Y", m_stick1.GetX());
//       frc::SmartDashboard::PutNumber("Right Hand Y", m_stick1.GetY());
//       frc::SmartDashboard::PutNumber("Left Hand X", m_stick1.GetZ());
      
//       bool noJoystick = false;
//       bool noJoystickX = false;
//       bool noJoystickY = false;
//       bool noJoystickRot = false;
//       double safeX = m_stick1.GetX();
//       if(fabs(safeX)<0.15) {
//           safeX=0;
//           noJoystickX = true;
//       }
//       double safeY =  m_stick1.GetY();
//       if(fabs(safeY)<0.15) { 
//           safeY=0;
//           noJoystickY = true;
//       }
//       double safeRot = m_stick1.GetZ();
//       if(fabs(safeRot)<0.5) {
//           safeRot=0;
//           noJoystickRot = true;
//       }
//       noJoystick = noJoystickX && noJoystickY && noJoystickRot;

//       frc::SmartDashboard::PutNumber("noJoystick val ", noJoystick);
      
//       // std::cout << "Sam Debug" << safeX << "," << safeY << "," << safeRot << std::endl;
      
//       m_drive.Drive(units::meters_per_second_t(
//                         -safeY * AutoConstants::kMaxSpeed),
//                     units::meters_per_second_t(
//                         -safeX * AutoConstants::kMaxSpeed),
//                     units::radians_per_second_t(
//                         -safeRot * PI),
//                     false,
//                     noJoystick);
//       // m_drive.Drive(units::meters_per_second_t(0),
//       // units::meters_per_second_t(1),
//       // units::radians_per_second_t(0),
//       // false);
//     }, {&m_drive}));
// }

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_stick1, 7).OnTrue(m_drive.SetDriveSlow(true));
  frc2::JoystickButton(&m_stick1, 7).OnFalse(m_drive.SetDriveSlow(false));
  frc2::JoystickButton(&m_stick1, 1).OnTrue(m_drive.ButtonZeroHeading());

  frc2::JoystickButton(&m_stick1, 2).OnTrue(m_drive.ConfigOdometry());


  frc2::JoystickButton(&m_xbox, 5).OnTrue(m_elevator.ClawOpenCommand());
  frc2::JoystickButton(&m_xbox, 6).OnFalse(m_elevator.ClawCloseCommand());
  frc2::JoystickButton(&m_stick1, 4).OnTrue(m_elevator.ClawOpenCommand()); //on joystick
  frc2::JoystickButton(&m_stick1, 3).OnFalse(m_elevator.ClawCloseCommand()); //on  joystick

  //frc2::JoystickButton(&m_stick1, 14).OnTrue(m_elevator.SetPlaceHighState())
  //frc2::JoystickButton(&m_stick1, 15).OnTrue(m_elevator.SetPlaceMidState());
  //frc2::JoystickButton(&m_stick1, 16).OnTrue(m_elevator.SetPlaceLowState());
  frc2::JoystickButton(&m_stick1, 5).OnTrue(m_elevator.SetManualElevatorState());//need to change

}
/*
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
      {frc::Translation2d{0.3_m, 0_m} , frc::Translation2d{0.6_m, 0_m}, frc::Translation2d{1_m, 0_m}},
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
*/

frc2::CommandPtr RobotContainer::DrivePath1(DriveSubsystem &m_drive){
      PathPlannerTrajectory examplePath = PathPlanner::loadPath("ChargeStation1", PathConstraints(3_mps, 1_mps_sq));


      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  SwerveAutoBuilder autoBuilder(
      [&m_drive]() { return m_drive.GetPose(); }, // Function to supply current robot pose
      [&m_drive](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(ModuleConstants::kPModuleDriveController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(ModuleConstants::kPModuleTurningController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [&m_drive](frc::ChassisSpeeds speeds) { m_drive.Drive(speeds.vx, speeds.vy, speeds.omega, false, false); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      { &m_drive }, // Drive requirements, usually just a single drive subsystem
      false // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );

  return autoBuilder.followPath(examplePath); //examplePathCmdPtr
  };

frc2::CommandPtr RobotContainer::DrivePath2(DriveSubsystem &m_drive){
      PathPlannerTrajectory examplePath = PathPlanner::loadPath("ChargeStation2", PathConstraints(3_mps, 1_mps_sq), true);


      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  SwerveAutoBuilder autoBuilder(
      [&m_drive]() { return m_drive.GetPose(); }, // Function to supply current robot pose
      [&m_drive](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(ModuleConstants::kPModuleDriveController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(ModuleConstants::kPModuleTurningController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [&m_drive](frc::ChassisSpeeds speeds) { m_drive.Drive(speeds.vx, speeds.vy, speeds.omega, false, false); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      { &m_drive }, // Drive requirements, usually just a single drive subsystem
      false // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );

  return autoBuilder.followPath(examplePath); //examplePathCmdPtr
  };


frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Runs the chosen command in autonomous
  // SequentialAuto test = SequentialAuto(m_drive);

    std::vector<std::unique_ptr<Command>> commands;


  commands.emplace_back(Drive1.get());
  commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Finished Path1" << std::endl;}));

  commands.emplace_back(Drive2.get());
  commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"In Position for charging station" << std::endl;}));
  // commands.emplace_back(AutoCmd);


  // // auto group = SequentialCommandGroup(std::move(commands));

  return new SequentialCommandGroup(std::move(commands));

  // return Drive1.get();
  // return Drive2.get();

  // return m_chooser.GetSelected();
}

  double RobotContainer::GetHeading(){

    return (double)m_drive.GetHeading();
  }

  double RobotContainer::GetOdometry(){
    frc::Pose2d Pose = m_drive.GetPose();
    return (double)Pose.Rotation().Degrees();
    
    // m_drive.GetPose();
  };

  // double RobotContainer::GetRotation(){}

    // int m_state = 0;
  
  // return AutoCmd;

/* m state = 0
 while loop, while m state != 1
 drive robot forward, if angle is greater than 15, m state = 1
 while m state != 2, run while loop 2, if angle is less than 5, m state = 2
 m drive 00 */
  void RobotContainer::ZeroHeading(){
    m_drive.ZeroHeading();
}

  void RobotContainer::ConfigMotorControllers(){
    m_drive.ConfigMotorControllers();
  }

  void RobotContainer::ResetOdometry(){
    // m_drive.ResetOdometry(frc::Pose2d{4.0_m, 4.5_m, 180_deg}); //SimpleStation
    // m_drive.ResetOdometry(frc::Pose2d{3.60_m, 0.75_m, 180_deg}); for ChargeStation1Place-ChargeStation2 paths
    m_drive.ResetOdometry(frc::Pose2d{4.4_m, 0.75_m, 180_deg}); //ChargeStation1
  }

