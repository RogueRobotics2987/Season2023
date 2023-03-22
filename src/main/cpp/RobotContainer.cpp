// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>



using namespace DriveConstants;
using namespace pathplanner;


RobotContainer::RobotContainer() {
  
  frc::SmartDashboard::PutString("AllienceSelector", "Blue");
  frc::SmartDashboard::PutNumber("PathSelector", 0);

  frc::SmartDashboard::PutData(&m_elevator);

  ConfigMotorControllers();
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  m_elevator.SetDefaultCommand(ElevatorCmd(m_elevator, m_xbox, m_newXbox));
  m_compressor.SetDefaultCommand(BeginCompressor(m_compressor));

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.



  m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      bool noJoystick = false;
      double safeX = Deadzone(m_newXbox.GetLeftX(), true);
      double safeY =  Deadzone(m_newXbox.GetLeftY(), true);
      double safeRot = Deadzone(m_newXbox.GetRightX(), true);
      bool fieldOrientated; 

      if (m_newXbox.GetRawAxis(3)> 0.15){
        fieldOrientated = false;
      }
      if (m_newXbox.GetRawAxis(3)< 0.15){
        fieldOrientated = true;
      }
      if((safeX == 0) && (safeY == 0) && (safeRot == 0)) {
        noJoystick = true;
      }
      m_drive.Drive(units::meters_per_second_t(
                    -safeY * AutoConstants::kMaxSpeed),
                    units::meters_per_second_t(
                    -safeX * AutoConstants::kMaxSpeed),
                    units::radians_per_second_t(
                    -safeRot * PI * 2.25),//was 1.5
                    fieldOrientated,
                    noJoystick);
      // m_drive.Drive(units::meters_per_second_t(0),
      // units::meters_per_second_t(1),
      // units::radians_per_second_t(0),
      // false);
    }, {&m_drive}));
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
  //needs to be changed to xbox
  // frc2::JoystickButton(&m_xbox, 7).OnTrue(m_drive.SetDriveSlow(true));
  // frc2::JoystickButton(&m_xbox, 7).OnFalse(m_drive.SetDriveSlow(false));
  //frc2::JoystickButton(&m_stick1, 1).OnTrue(m_drive.ButtonZeroHeading());

  //frc2::JoystickButton(&m_stick1, 2).OnTrue(m_drive.ConfigOdometry());

  frc2::JoystickButton(&m_newXbox, 2).WhileTrue(m_drive.Twitch(true));
  frc2::JoystickButton(&m_newXbox, 4).WhileTrue(m_drive.Twitch(false));


  frc2::JoystickButton(&m_xbox, 5).OnTrue(m_elevator.ClawCloseCommand());
  frc2::JoystickButton(&m_xbox, 6).OnFalse(m_elevator.ClawOpenCommand());
  frc2::JoystickButton(&m_newXbox, 3).OnTrue(m_elevator.ClawCloseCommand()); //Button X
  frc2::JoystickButton(&m_newXbox, 1).OnFalse(m_elevator.ClawOpenCommand()); //Button A

 // frc2::JoystickButton(&m_xbox, 7).OnTrue(m_elevator.SetManualElevatorState());//need to change
  //frc2::JoystickButton(&m_xbox, 7).WhileTrue(m_elevator.SetArmPos(-90));
  //frc2::JoystickButton(&m_xbox, 7).WhileTrue(m_elevator.SetVertPos(70.4)); //or 52
  //frc2::JoystickButton(&m_xbox, 7).WhileTrue(m_elevator.SetElevatorPos(-90, 70.4));

  //frc2::JoystickButton(&m_xbox, 8).WhileTrue(m_elevator.SetArmPos(-45));
  //frc2::JoystickButton(&m_xbox, 8).WhileTrue(m_elevator.SetVertPos(106.6));
  //currently 104 because I am worried about hitting the limit swtich too fast
  // frc2::JoystickButton(&m_xbox, 8).WhileTrue(m_elevator.SetElevatorPos(-45, 104));//was 106.6
  frc2::JoystickButton(&m_xbox, 4).WhileTrue(PlaceHighCmd); //Button Y
  //frc2::JoystickButton(&m_xbox, 2).WhileTrue(PickupCmd); //Button B //old pickup cmd
  frc2::JoystickButton(&m_xbox, 1).WhileTrue(PickupTipCmd); //Button A
  frc2::JoystickButton(&m_xbox, 2).WhileTrue(PickupWithBumpersIntoSubstation); //button B
  // frc2::JoystickButton(&m_newXbox, 3).WhileTrue(PlaceMidCmd); //Button X
  // frc2::JoystickButton(&m_newXbox, 8).WhileTrue(PlaceLowCmd); //Small button right

  //frc2::JoystickButton(&m_newXbox, 7).OnTrue(m_drive.FieldOrientatedTrue());
  //frc2::JoystickButton(&m_newXbox, 8).OnTrue(m_drive.FieldOrientatedFalse());
  frc2::JoystickButton(&m_newXbox, 5).OnTrue(m_drive.ZeroHeading());
  

  frc2::JoystickButton(&m_xbox, 7).OnTrue(m_lights.CubeDesired()); //blue
  frc2::JoystickButton(&m_xbox, 3).OnTrue(m_lights.AllianceColorCmdPtr()); //yellow
  // frc2::JoystickButton(&m_xbox, 3).OnTrue(m_lights.CubeDesired()); //Actually red, now left center
  // frc2::JoystickButton(&m_xbox, 3).OnTrue(m_lights.RedColor()); //X-is actually blue
  frc2::JoystickButton(&m_xbox, 8).OnTrue(m_lights.ConeDesired()); //red

  // frc::DriverStation::Alliance::kRed


}


//Red Paths

frc2::CommandPtr RobotContainer::DriveCrgStnRed1(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("ChargeStation1Red", PathPlanner::getConstraintsFromPath("ChargeStation1Red"), true);
  std::cout<<"ChargeStation1Red"<<std::endl;

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

frc2::CommandPtr RobotContainer::DriveCrgStnRed2(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("ChargeStation2Red", PathPlanner::getConstraintsFromPath("ChargeStation2Red"), true);
  std::cout<<"ChargeStation2Red"<<std::endl;

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

//Blue paths

frc2::CommandPtr RobotContainer::DriveCrgStnBlue1(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("ChargeStation1Blue", PathPlanner::getConstraintsFromPath("ChargeStation1Blue"), true);
  std::cout<<"ChargeStation1Blue"<<std::endl;

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

frc2::CommandPtr RobotContainer::DriveCrgStnBlue2(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("ChargeStation2Blue", PathPlanner::getConstraintsFromPath("ChargeStation2Blue"), true);
  std::cout<<"ChargeStation2Blue"<<std::endl;

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

  frc2::CommandPtr RobotContainer::PlaceDriveCrgStnBlue1(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("PlaceChargeStation1Blue", PathPlanner::getConstraintsFromPath("PlaceChargeStation1Blue"), true);
  std::cout<<"PlaceChargeStation1Blue"<<std::endl;

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

  frc2::CommandPtr RobotContainer::PlaceDriveCrgStnRed1(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("PlaceChargeStation1Red", PathPlanner::getConstraintsFromPath("PlaceChargeStation1Red"), true);
  std::cout<<"PlaceChargeStation1Red"<<std::endl;

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

  frc2::CommandPtr RobotContainer::ConeBalanceBlue(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("ConeAndBalanceBlue", PathPlanner::getConstraintsFromPath("ConeAndBalanceBlue"), true);
  std::cout<<"ConeAndBalanceBLue"<<std::endl;

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

frc2::CommandPtr RobotContainer::ConeBalanceRed(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("ConeAndBalanceRed", PathPlanner::getConstraintsFromPath("ConeAndBalanceRed"), true);
  std::cout<<"ConeAndBalanceRed"<<std::endl;

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
  
  frc2::CommandPtr RobotContainer::RedLeave1(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("RedLeave1", PathPlanner::getConstraintsFromPath("RedLeave1"), true);
  std::cout<<"RedLeave1"<<std::endl;

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
  
  frc2::CommandPtr RobotContainer::RedLeave2(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("RedLeave2", PathPlanner::getConstraintsFromPath("RedLeave2"), true);
  std::cout<<"RedLeave2"<<std::endl;

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

  frc2::CommandPtr RobotContainer::BlueLeave1(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("BlueLeave1", PathPlanner::getConstraintsFromPath("BlueLeave1"), true);
  std::cout<<"BlueLeave1"<<std::endl;

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

  frc2::CommandPtr RobotContainer::BlueLeave2(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("BlueLeave2", PathPlanner::getConstraintsFromPath("BlueLeave2"), true);
  std::cout<<"BlueLeave2"<<std::endl;

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

  frc2::CommandPtr RobotContainer::Red2Place1(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("Red2place1", PathPlanner::getConstraintsFromPath("Red2place1"), true);
  std::cout<<"Red2Place1"<<std::endl;

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  SwerveAutoBuilder autoBuilder(
      [&m_drive]() { return m_drive.GetPose(); }, // Function to supply current robot pose
      [&m_drive](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(AutoConstants::kPXYController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(AutoConstants::kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [&m_drive](frc::ChassisSpeeds speeds) { m_drive.Drive(speeds.vx, speeds.vy, speeds.omega, false, false); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      { &m_drive }, // Drive requirements, usually just a single drive subsystem
      false // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );

  return autoBuilder.followPath(examplePath); //examplePathCmdPtr
  };

  frc2::CommandPtr RobotContainer::Red2Place2(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("Red2place2", PathPlanner::getConstraintsFromPath("Red2place2"), true);
  std::cout<<"Red2Place2"<<std::endl;

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  SwerveAutoBuilder autoBuilder(
      [&m_drive]() { return m_drive.GetPose(); }, // Function to supply current robot pose
      [&m_drive](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(AutoConstants::kPXYController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(AutoConstants::kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [&m_drive](frc::ChassisSpeeds speeds) { m_drive.Drive(speeds.vx, speeds.vy, speeds.omega, false, false); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      { &m_drive }, // Drive requirements, usually just a single drive subsystem
      false // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );

  return autoBuilder.followPath(examplePath); //examplePathCmdPtr
  };

  frc2::CommandPtr RobotContainer::Red2Place3(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("Red2place3", PathPlanner::getConstraintsFromPath("Red2place3"), true);
  std::cout<<"Red2Place3"<<std::endl;

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  SwerveAutoBuilder autoBuilder(
      [&m_drive]() { return m_drive.GetPose(); }, // Function to supply current robot pose
      [&m_drive](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(AutoConstants::kPXYController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(AutoConstants::kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [&m_drive](frc::ChassisSpeeds speeds) { m_drive.Drive(speeds.vx, speeds.vy, speeds.omega, false, false); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      { &m_drive }, // Drive requirements, usually just a single drive subsystem
      false // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );

  return autoBuilder.followPath(examplePath); //examplePathCmdPtr
  };

  frc2::CommandPtr RobotContainer::Forward1(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("Forward1", PathPlanner::getConstraintsFromPath("Forward1"), true);
  std::cout<<"Forward1"<<std::endl;

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  SwerveAutoBuilder autoBuilder(
      [&m_drive]() { return m_drive.GetPose(); }, // Function to supply current robot pose
      [&m_drive](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(AutoConstants::kPXYController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(AutoConstants::kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [&m_drive](frc::ChassisSpeeds speeds) { m_drive.Drive(speeds.vx, speeds.vy, speeds.omega, false, false); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      { &m_drive }, // Drive requirements, usually just a single drive subsystem
      false // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );

  return autoBuilder.followPath(examplePath); //examplePathCmdPtr
  };

  frc2::CommandPtr RobotContainer::Forward2(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("Forward2", PathPlanner::getConstraintsFromPath("Forward2"), true);
  std::cout<<"Forward2"<<std::endl;

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  SwerveAutoBuilder autoBuilder(
      [&m_drive]() { return m_drive.GetPose(); }, // Function to supply current robot pose
      [&m_drive](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(AutoConstants::kPXYController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(AutoConstants::kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [&m_drive](frc::ChassisSpeeds speeds) { m_drive.Drive(speeds.vx, speeds.vy, speeds.omega, false, false); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      { &m_drive }, // Drive requirements, usually just a single drive subsystem
      false // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );

  return autoBuilder.followPath(examplePath); //examplePathCmdPtr
  };

  frc2::CommandPtr RobotContainer::Slide1(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("Slide1", PathPlanner::getConstraintsFromPath("Slide1"), true);
  std::cout<<"Slide1"<<std::endl;

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  SwerveAutoBuilder autoBuilder(
      [&m_drive]() { return m_drive.GetPose(); }, // Function to supply current robot pose
      [&m_drive](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(AutoConstants::kPXYController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(AutoConstants::kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [&m_drive](frc::ChassisSpeeds speeds) { m_drive.Drive(speeds.vx, speeds.vy, speeds.omega, false, false); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      { &m_drive }, // Drive requirements, usually just a single drive subsystem
      false // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );

  return autoBuilder.followPath(examplePath); //examplePathCmdPtr
  };

  frc2::CommandPtr RobotContainer::Slide2(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("Slide2", PathPlanner::getConstraintsFromPath("Slide2"), true);
  std::cout<<"Slide2"<<std::endl;

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  SwerveAutoBuilder autoBuilder(
      [&m_drive]() { return m_drive.GetPose(); }, // Function to supply current robot pose
      [&m_drive](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(AutoConstants::kPXYController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(AutoConstants::kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [&m_drive](frc::ChassisSpeeds speeds) { m_drive.DriveAutonomous(speeds.vx, speeds.vy, speeds.omega, false, false); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      { &m_drive }, // Drive requirements, usually just a single drive subsystem
      false // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );

  return autoBuilder.followPath(examplePath); //examplePathCmdPtr
  };

  frc2::CommandPtr RobotContainer::Twist2(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("Twist2", PathPlanner::getConstraintsFromPath("Twist2"), true);
  std::cout<<"Twist2"<<std::endl;

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  SwerveAutoBuilder autoBuilder(
      [&m_drive]() { return m_drive.GetPose(); }, // Function to supply current robot pose
      [&m_drive](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(AutoConstants::kPXYController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(AutoConstants::kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [&m_drive](frc::ChassisSpeeds speeds) { m_drive.DriveAutonomous(speeds.vx, speeds.vy, speeds.omega, false, false); }, // Output function that accepts field relative ChassisSpeeds
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

  int pathselector = frc::SmartDashboard::GetNumber("PathSelector", 0);
  std::string AllienceSelector = frc::SmartDashboard::GetString("AllienceSelector", "Red");

  //Red Path
  if(pathselector == 0 && AllienceSelector == "Red"){
    ResetOdometry();
    commands.emplace_back(DriveCrgStnRed1cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Finished Path1" << std::endl;}));
    commands.emplace_back(AutoZeroHeading.get());
    commands.emplace_back(DriveCrgStnRed2cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"In Position for charging station" << std::endl;}));
  }
  //Blue Path
  else if(pathselector == 0 && AllienceSelector == "Blue"){
    ResetOdometry();
    commands.emplace_back(DriveCrgStnBlue1cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Finished Path1" << std::endl;}));
    commands.emplace_back(AutoZeroHeading.get());
    commands.emplace_back(DriveCrgStnBlue2cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"In Position for charging station" << std::endl;}));    
  }
  //Red Charge Station
  else if(pathselector == 1 && AllienceSelector == "Red"){
    ResetOdometry();
    commands.emplace_back(DriveCrgStnRed1cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Finished Path1" << std::endl;}));
    commands.emplace_back(AutoZeroHeading.get());
    commands.emplace_back(DriveCrgStnRed2cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"In Position for charging station" << std::endl;}));
    commands.emplace_back(AutoCmd);
  }
  //Blue Charge station
  else if(pathselector == 1 && AllienceSelector ==  "Blue"){
    ResetOdometry();
    commands.emplace_back(DriveCrgStnBlue1cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Finished Path1" << std::endl;}));
    commands.emplace_back(AutoZeroHeading.get());
    commands.emplace_back(DriveCrgStnBlue2cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"In Position for charging station" << std::endl;}));  
    commands.emplace_back(AutoCmd);
  }
  else if(pathselector == 2 && AllienceSelector == "Red"){
    ResetOdometry();
    std::vector<std::unique_ptr<Command>> p1_commands;
    p1_commands.emplace_back(RetractCmdAuto);
    p1_commands.emplace_back(Leave1Red.get());
    m_drive.SetAngleAdjustment(180);
    commands.emplace_back(new PlaceAutoCmd(m_elevator, 104, -30, 160));
    commands.emplace_back(PlaceHighRace);
    commands.emplace_back(OpenClawCmd.get());
    commands.emplace_back(new TimerCMD(.5));
    commands.emplace_back(new frc2::ParallelCommandGroup(std::move(p1_commands)));    
  }
  else if(pathselector == 2 && AllienceSelector == "Blue"){
    ResetOdometry();
      std::vector<std::unique_ptr<Command>> p1_commands;
    p1_commands.emplace_back(RetractCmdAuto);
    p1_commands.emplace_back(Leave1Blue.get());
    m_drive.SetAngleAdjustment(180);
    commands.emplace_back(new PlaceAutoCmd(m_elevator, 104, -30, 160));
    commands.emplace_back(PlaceHighRace);
    commands.emplace_back(OpenClawCmd.get());
    commands.emplace_back(new TimerCMD(.5));
    commands.emplace_back(new frc2::ParallelCommandGroup(std::move(p1_commands)));      
  }
  else if(pathselector == 3 && AllienceSelector == "Red"){
    ResetOdometry();
    std::vector<std::unique_ptr<Command>> p1_commands;
    p1_commands.emplace_back(RetractCmdAuto);
    p1_commands.emplace_back(Leave2Red.get());
    m_drive.SetAngleAdjustment(180);
    commands.emplace_back(new PlaceAutoCmd(m_elevator, 104, -30, 160));
    commands.emplace_back(PlaceHighRace);
    commands.emplace_back(OpenClawCmd.get());
    commands.emplace_back(new TimerCMD(.5));
    commands.emplace_back(new frc2::ParallelCommandGroup(std::move(p1_commands)));        
  }
  else if(pathselector == 3 && AllienceSelector == "Blue"){
    ResetOdometry();
    std::vector<std::unique_ptr<Command>> p1_commands;
    p1_commands.emplace_back(RetractCmdAuto);
    p1_commands.emplace_back(Leave2Blue.get());
    // m_drive.SetAngleAdjustment(180);
    commands.emplace_back(new PlaceAutoCmd(m_elevator, 104, -30, 160));
    commands.emplace_back(PlaceHighRace);
    commands.emplace_back(OpenClawCmd.get());
    commands.emplace_back(new TimerCMD(.5));
    commands.emplace_back(new frc2::ParallelCommandGroup(std::move(p1_commands)));        
  }
  else if(pathselector == 4 && AllienceSelector == "Blue"){
    ResetOdometry();
    std::vector<std::unique_ptr<Command>> p1_commands;
    p1_commands.emplace_back(RetractCmdAuto);
    p1_commands.emplace_back(ConeBalanceBlueCmd.get());
    // m_drive.SetAngleAdjustment(180);
    commands.emplace_back(new PlaceAutoCmd(m_elevator, 104, -30, 160));
    commands.emplace_back(PlaceHighRace);
    commands.emplace_back(OpenClawCmd.get());
    commands.emplace_back(new TimerCMD(.5));
    // commands.emplace_back(RetractCmd);
    // commands.emplace_back(ConeBalanceBlueCmd.get());
    commands.emplace_back(new frc2::ParallelCommandGroup(std::move(p1_commands)));
    commands.emplace_back(AutoCmd);
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"cone" << std::endl;}));  
  }

  else if(pathselector == 4 && AllienceSelector == "Red"){
    ResetOdometry();
     std::vector<std::unique_ptr<Command>> p1_commands;
    p1_commands.emplace_back(RetractCmdAuto);
    p1_commands.emplace_back(ConeBalanceRedCmd.get());
    // m_drive.SetAngleAdjustment(180);
    commands.emplace_back(new PlaceAutoCmd(m_elevator, 104, -30, 160));
    // commands.emplace_back(new PlaceAutoCmd(m_elevator, 104, -60, 160));
    commands.emplace_back(PlaceHighRace);
    commands.emplace_back(OpenClawCmd.get());
    commands.emplace_back(new TimerCMD(.5));
    // commands.emplace_back(new frc2::ParallelCommandGroup(
    //   RetractCmd, (Command*)ConeBalanceRedCmd.get()
    // ));
    // commands.emplace_back(RetractCmd);
    // commands.emplace_back(ConeBalanceRedCmd.get());
    commands.emplace_back(new frc2::ParallelCommandGroup(std::move(p1_commands)));
    commands.emplace_back(AutoCmd);
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"cone" << std::endl;}));    
  }
  else if(pathselector == 5 && AllienceSelector == "Red"){
    ResetOdometry();
    // m_drive.SetAngleAdjustment(180);
    commands.emplace_back(new PlaceAutoCmd(m_elevator, 104, -30, 160));
    commands.emplace_back(PlaceHighRace);
    commands.emplace_back(OpenClawCmd.get());
    commands.emplace_back(new TimerCMD(.5));
    commands.emplace_back(RetractCmdAuto);  
    commands.emplace_back(PlaceDriveCrgStnRed1cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Finished Path1" << std::endl;}));
    commands.emplace_back(AutoZeroHeading.get());
    commands.emplace_back(DriveCrgStnRed2cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"In Position for charging station" << std::endl;}));
    commands.emplace_back(AutoCmd);
  }
  else if(pathselector == 5 && AllienceSelector == "Blue"){
    ResetOdometry();
    // m_drive.SetAngleAdjustment(180);
    commands.emplace_back(new PlaceAutoCmd(m_elevator, 104, -30, 160));
    commands.emplace_back(PlaceHighRace);
    commands.emplace_back(OpenClawCmd.get());
    commands.emplace_back(new TimerCMD(.5));
    commands.emplace_back(RetractCmdAuto);
    commands.emplace_back(PlaceDriveCrgStnBlue1cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Finished Path1" << std::endl;}));
    commands.emplace_back(AutoZeroHeading.get());
    commands.emplace_back(DriveCrgStnBlue2cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"In Position for charging station" << std::endl;}));  
    commands.emplace_back(AutoCmd);
  }
  else if(pathselector == 6 && AllienceSelector == "Red"){
    ResetOdometry();
    std::vector<std::unique_ptr<Command>> s1_commands;
    s1_commands.emplace_back(RetractPickupCmdAuto);
    s1_commands.emplace_back(new TimerCMD(.55));    
    s1_commands.emplace_back(PickupFlatAuto);
    //s1 Retracts Arm back then lowers to pickup cube
    std::vector<std::unique_ptr<Command>> p1_commands;
    p1_commands.emplace_back(new frc2::SequentialCommandGroup(std::move(s1_commands)));
    p1_commands.emplace_back(Red2Place1Cmd.get());   
    //p1 moves away from place and does s1 at the same time
    // std::vector<std::unique_ptr<Command>> p2_commands;
    // p2_commands.emplace_back(RetractCmdAuto);
    // p2_commands.emplace_back(Red2Place3Cmd.get());
    //p2 retracts and moves to charge at same time

    // std::vector<std::unique_ptr<Command>> p3_commands;
    // p3_commands.emplace_back(PickupFlatAuto);
    // p3_commands.emplace_back(Red2Place2Cmd.get());          

    // m_drive.SetAngleAdjustment(180);
    commands.emplace_back(new PlaceAutoCmd(m_elevator, 104, -30, 160));
    commands.emplace_back(PlaceHighRace);
    commands.emplace_back(OpenClawCmd.get());
    commands.emplace_back(new TimerCMD(.2));
    commands.emplace_back(new frc2::ParallelCommandGroup(std::move(p1_commands)));
    // commands.emplace_back(RetractPickupCmdAuto);  //change to a elevator vertical with claw up
    // commands.emplace_back(Red2Place1Cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Finished Path1" << std::endl;}));
    commands.emplace_back(CloseClawCmd.get());
    commands.emplace_back(RetractPickupCmdAuto2);
    // commands.emplace_back(AutoZeroHeading.get());
    commands.emplace_back(Red2Place2Cmd.get());
    commands.emplace_back(PlaceHighRace2);
    commands.emplace_back(PlaceHighRace3);
    commands.emplace_back(new TimerCMD(.25));
    commands.emplace_back(OpenClawCmd.get());
    commands.emplace_back(new TimerCMD(.25));
    // commands.emplace_back(new frc2::ParallelCommandGroup(std::move(p2_commands)));
    commands.emplace_back(RetractCmdAuto); 
    // commands.emplace_back(Red2Place3Cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"EndRed2" << std::endl;}));  
    // commands.emplace_back(AutoCmd);
  }
  else if(pathselector == 0 && AllienceSelector == "Test"){
    ResetOdometry();
    commands.emplace_back(Forward1Cmd.get()); 
    commands.emplace_back(Forward2Cmd.get()); 
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Finished" << std::endl;}));  
  }
  else if(pathselector == 1 && AllienceSelector == "Test"){
    ResetOdometry();
    commands.emplace_back(Forward1Cmd.get()); 
    commands.emplace_back(Slide1Cmd.get()); 
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Half" << std::endl;}));  
    commands.emplace_back(Slide2Cmd.get()); 
    commands.emplace_back(Forward2Cmd.get()); 
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Finished" << std::endl;}));  
  }
  else if(pathselector == 2 && AllienceSelector == "Test"){
    ResetOdometry();
    commands.emplace_back(Twist2Cmd.get()); 
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Finished" << std::endl;}));  
  }
  else{
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Do Nothing" << std::endl;}));
  }
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
    int pathselector = frc::SmartDashboard::GetNumber("PathSelector", 0);
    std::string AllienceSelector = frc::SmartDashboard::GetString("AllienceSelector", "Red");

    // pathselector = 1;

    if((pathselector == 0 || pathselector == 1) && AllienceSelector == "Red"){
      m_drive.ResetOdometry(frc::Pose2d{12.2_m, 0.75_m, 180_deg}); //ChargeStation1Red
      std::cout<<"OdometryRed"<<std::endl;
    }
    else if((pathselector == 0 || pathselector == 1) && AllienceSelector == "Blue"){
    //if(pathselector == 1){
      m_drive.ResetOdometry(frc::Pose2d{4.4_m, 0.75_m, 0_deg}); //ChargeStation1Blue
      std::cout<<"OdometryBlue"<<std::endl;
    }
    else if(pathselector == 2 && AllienceSelector == "Red"){ //Leave1
      m_drive.ResetOdometry(frc::Pose2d{12.2_m, 0.75_m, 0_deg});
      std::cout<<"OdometryPlaceRed"<<std::endl;
    }
    else if(pathselector == 2 && AllienceSelector == "Blue"){ //Place ChargeStation blue
      m_drive.ResetOdometry(frc::Pose2d{4.35_m, 0.75_m, 180_deg});
      std::cout<<"OdometryPlaceBlue"<<std::endl;

    }    
    else if(pathselector == 3 && AllienceSelector == "Red"){ //Leave2
      m_drive.ResetOdometry(frc::Pose2d{13.75_m, 4.75_m, 0_deg});
      std::cout<<"OdometryPlaceRed"<<std::endl;
    }
    else if(pathselector == 3 && AllienceSelector == "Blue"){ //Place ChargeStation blue
      m_drive.ResetOdometry(frc::Pose2d{2.8_m, 4.75_m, 180_deg});
      std::cout<<"OdometryPlaceBlue"<<std::endl;

    }
    else if(pathselector == 4 && AllienceSelector == "Blue"){ //Place ChargeStation blue
      m_drive.ResetOdometry(frc::Pose2d{1.9_m, 4.94_m, 180_deg});
      std::cout<<"OdometryCone"<<std::endl;
    }

    else if(pathselector == 4 && AllienceSelector == "Red"){ //Place ChargeStation blue
      m_drive.ResetOdometry(frc::Pose2d{14.65_m, 4.94_m, 0_deg});
      std::cout<<"OdometryCone"<<std::endl;
    }    
    else if(pathselector == 5 && AllienceSelector == "Blue"){ //Place ChargeStation blue
      m_drive.ResetOdometry(frc::Pose2d{1.9_m, 0.5_m, 180_deg});
      std::cout<<"OdometryCone"<<std::endl;
    }    
    else if(pathselector == 5 && AllienceSelector == "Red"){ //Place ChargeStation blue
      m_drive.ResetOdometry(frc::Pose2d{14.6_m, 0.5_m, 0_deg});
      std::cout<<"OdometryCone"<<std::endl;
    }
    else if(pathselector == 6 && AllienceSelector == "Red"){ //Place ChargeStation blue
      m_drive.ResetOdometry(frc::Pose2d{14.65_m, 5_m, 0_deg});
      std::cout<<"Red6"<<std::endl;
    }
    else if((pathselector == 0 || pathselector == 1 || pathselector == 2) && AllienceSelector == "Test"){
      m_drive.ResetOdometry(frc::Pose2d{5_m, 5_m, 0_deg});
    }
    // else if(pathselector == 5 && AllienceSelector == "Red"){ //Place ChargeStation blue
    //   m_drive.ResetOdometry(frc::Pose2d{14.6_m, 0.5_m, 0_deg});
    //   std::cout<<"OdometryCone"<<std::endl;
    // }
    else{
      std::cout<<"Do Nothing For Odometry, Paramaters incorrect" << std::endl;
    }
  }

  float RobotContainer::Deadzone(float x, bool rotation) {
  if((x < 0.1) && (x > -0.1)) {
    x=0;
  }
  if(rotation == true){
    if(x >= 0.1) {
      x = x - 0.1;
    }
    else if(x <= -0.1) {
      x = x + 0.1;
   }
  }
  return(x);
}