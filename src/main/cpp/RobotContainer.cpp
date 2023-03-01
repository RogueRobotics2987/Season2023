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
      double safeX = Deadzone(m_newXbox.GetLeftX());
      double safeY =  Deadzone(m_newXbox.GetLeftY());
      double safeRot = Deadzone(m_newXbox.GetRightX());
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
                    -safeRot * PI * 1.5),
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

  //frc2::JoystickButton(&m_stick1, 14).OnTrue(m_elevator.SetPlaceHighState())
  //frc2::JoystickButton(&m_stick1, 15).OnTrue(m_elevator.SetPlaceMidState());
  //frc2::JoystickButton(&m_stick1, 16).OnTrue(m_elevator.SetPlaceLowState());
 // frc2::JoystickButton(&m_xbox, 7).OnTrue(m_elevator.SetManualElevatorState());//need to change
  //frc2::JoystickButton(&m_xbox, 7).WhileTrue(m_elevator.SetArmPos(-90));
  //frc2::JoystickButton(&m_xbox, 7).WhileTrue(m_elevator.SetVertPos(70.4)); //or 52
  frc2::JoystickButton(&m_xbox, 7).WhileTrue(m_elevator.SetElevatorPos(-90, 70.4));

  //frc2::JoystickButton(&m_xbox, 8).WhileTrue(m_elevator.SetArmPos(-45));
  //frc2::JoystickButton(&m_xbox, 8).WhileTrue(m_elevator.SetVertPos(106.6));
  //currently 104 because I am worried about hitting the limit swtich too fast
  frc2::JoystickButton(&m_xbox, 8).WhileTrue(m_elevator.SetElevatorPos(-45, 104));//was 106.6

  frc2::JoystickButton(&m_newXbox, 7).OnTrue(m_drive.FieldOrientatedTrue());
  frc2::JoystickButton(&m_newXbox, 8).OnTrue(m_drive.FieldOrientatedFalse());
  frc2::JoystickButton(&m_newXbox, 5).OnTrue(m_drive.ZeroHeading());
  

  frc2::JoystickButton(&m_xbox, 1).OnTrue(m_lights.ConeDesired());
  frc2::JoystickButton(&m_xbox, 2).OnTrue(m_lights.CubeDesired());
  frc2::JoystickButton(&m_xbox, 3).OnTrue(m_lights.RedColor());
  frc2::JoystickButton(&m_xbox, 4).OnTrue(m_lights.BlueColor());
}


//Red Paths

frc2::CommandPtr RobotContainer::DriveCrgStnRed1(DriveSubsystem &m_drive){

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("ChargeStation1Red", PathConstraints(3_mps, 1_mps_sq), true);
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

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("ChargeStation2Red", PathConstraints(3_mps, 1_mps_sq), true);
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

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("ChargeStation1Blue", PathConstraints(3_mps, 1_mps_sq), true);
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

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("ChargeStation2Blue", PathConstraints(3_mps, 1_mps_sq), true);
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

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("PlaceChargeStation1Blue", PathConstraints(3_mps, 1_mps_sq), true);
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

  PathPlannerTrajectory examplePath = PathPlanner::loadPath("PlaceChargeStation1Red", PathConstraints(3_mps, 1_mps_sq), true);
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
    //TODO move elevator to mid then place
    //arm must be negative
    commands.emplace_back(new frc2::InstantCommand([this] {m_elevator.SetArmPos(-90);}));
    commands.emplace_back(new frc2::InstantCommand([this] {m_elevator.ClawOpenCommand();}));
    commands.emplace_back(new frc2::InstantCommand([this] {m_elevator.SetArmPos(0);}));    
    commands.emplace_back(PlaceDriveCrgStnRed1cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Finished Path1" << std::endl;}));
    commands.emplace_back(AutoZeroHeading.get());
    commands.emplace_back(DriveCrgStnRed2cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"In Position for charging station" << std::endl;}));
    commands.emplace_back(AutoCmd);
  }
  else if(pathselector == 2 && AllienceSelector == "Blue"){
    ResetOdometry();
    //TODO move elevator to mid then place
    commands.emplace_back(new frc2::InstantCommand([this] {m_elevator.SetArmPos(-90);}));
    commands.emplace_back(new frc2::InstantCommand([this] {m_elevator.ClawOpenCommand();}));
    commands.emplace_back(new frc2::InstantCommand([this] {m_elevator.SetArmPos(0);}));        
    commands.emplace_back(PlaceDriveCrgStnBlue1cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Finished Path1" << std::endl;}));
    commands.emplace_back(AutoZeroHeading.get());
    commands.emplace_back(DriveCrgStnBlue2cmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"In Position for charging station" << std::endl;}));  
    commands.emplace_back(AutoCmd);
  }
  else if(pathselector == 3){
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Elevator Move" << std::endl;}));  
    // commands.emplace_back(new frc2::InstantCommand([this] {TimerCMD(0.5);}));    
    commands.emplace_back(OpenClawCmd.get());
    // commands.emplace_back(new frc2::InstantCommand([this] {m_elevator.SetArmPos(0);}));
    // commands.emplace_back(CloseClawCmd.get());
  }
  else if(pathselector == 4){
    commands.emplace_back(OpenClawCmd.get());    
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Claw Open" << std::endl;}));
    commands.emplace_back(SetHighCmd.get());            
    // commands.emplace_back(TiltSpeedCmd.get());    
    // commands.emplace_back(OpenClawCmd.get());
    commands.emplace_back(new frc2::InstantCommand([this] {std::cout<<"Finished" << std::endl;}));    
    // tilt to end then after 2 seconds open claw to drop
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
    else if(pathselector == 2 && AllienceSelector == "Red"){ //Place charge station red
      m_drive.ResetOdometry(frc::Pose2d{14.6_m, 0.5_m, 0_deg});
      std::cout<<"OdometryPlaceRed"<<std::endl;
    }
    else if(pathselector == 2 && AllienceSelector == "Blue"){ //Place ChargeStation blue
      m_drive.ResetOdometry(frc::Pose2d{1.9_m, 0.5_m, 180_deg});
      std::cout<<"OdometryPlaceBlue"<<std::endl;

    }
    else{
      std::cout<<"Do Nothing For Odometry, Paramaters incorrect" << std::endl;
    }
  }

  float RobotContainer::Deadzone(float x) {
  if((x < 0.1) && (x > -0.1)) {
    x=0;
  }
  else if(x >= 0.1) {
    x = x - 0.1;
  }
  else if(x <= -0.1) {
    x = x + 0.1;
  }
  return(x);
}