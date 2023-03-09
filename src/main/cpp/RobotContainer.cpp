// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <iostream>
#include <frc/controller/PIDController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>

#include "Constants.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
    m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
            if(controller.GetAButtonPressed()) {
              // if(m_drive.ZeroSwervePosition()) {
              //   m_drive.ResetEncoders();
              // }
            } else {
              double x = controller.GetLeftX();
              double y = controller.GetLeftY();

              // zero out axes if they fall within deadzone
              if (x > -DriveConstants::kDriveDeadzone && x < DriveConstants::kDriveDeadzone)
                  x = 0.0;
              if (y > -DriveConstants::kDriveDeadzone && y < DriveConstants::kDriveDeadzone)
                  y = 0.0;

              // put speeds through a polynomial to smooth out joystick input
              // check the curve out here: https://www.desmos.com/calculator/65tpwhxyai the range between 0.0 to 1.0 is used for the motors
              // change driveCurveExtent to modify curve strength
              float xSpeed = DriveConstants::kDriveCurveExtent * pow(x, 3) + (1 - DriveConstants::kDriveCurveExtent) * x;
              float ySpeed = DriveConstants::kDriveCurveExtent * pow(y, 3) + (1 - DriveConstants::kDriveCurveExtent) * y;
              m_drive.Drive(
                units::meters_per_second_t{ySpeed * 4.0},
                units::meters_per_second_t{xSpeed * -4.0},
                units::degrees_per_second_t{controller.GetRightX() * 150}, false);
              // m_drive.Drive(
              //   units::meters_per_second_t{xSpeed * 4.0},
              //   units::meters_per_second_t{ySpeed * -4.0},
              //   units::degrees_per_second_t{controller.GetRightX() * 150}, false);
            }
      },
      {&m_drive}));
    
    intake.SetDefaultCommand(frc2::RunCommand(
      [this] {
          intake.SetState(IntakeConstants::kPowerMode);
          double speed = controller.GetRightTriggerAxis() - controller.GetLeftTriggerAxis();
          if(speed > 0.1 || speed < -0.1) intake.SetPower(speed);
          if(controller.GetRightBumperPressed()) intake.SetPower(0.07);
          if(controller.GetLeftBumperPressed()) intake.SetPower(0.0);
          int pov = controller.GetPOV();
          switch(pov) {
            case 0: 
              intake.SetPosition(IntakeConstants::kHighDropoffPosition);
              break;
            case 90:
              intake.SetPosition(IntakeConstants::kMidDropoffPosition);
              break;
            case 180:
              intake.SetPosition(IntakeConstants::kFloorPickupPosition);
              break;
          }

          if(controller.GetBackButton()) intake.SetPosition(IntakeConstants::kStartPosition);
      },
      {&intake}));

    elevator.SetDefaultCommand(frc2::RunCommand(
      [this] {
            // elevator.SetTargetPosition(controller.GetLeftTriggerAxis() * 30000);
            // int state = elevator.GetState();
            // if(controller.GetAButtonPressed()) {
            //   if(state == ElevatorConstants::kPositionMode) elevator.SetState(ElevatorConstants::kOff);
            //   else if(state == ElevatorConstants::kOff) elevator.SetState(ElevatorConstants::kPositionMode);
            // }
            // elevator.SetState(ElevatorConstants::kPositionMode);
            // elevator.SetTargetPosition(SmartDashboard::GetNumber("elevatorPos", 0));
            elevator.SetState(ElevatorConstants::kPositionMode);
            int pov = controller.GetPOV();
            switch(pov) {
            case 0: 
              elevator.SetTargetPosition(ElevatorConstants::kHighDropoffPosition);
              break;
            case 90:
              elevator.SetTargetPosition(ElevatorConstants::kMidDropoffPosition);
              break;
            case 180:
              elevator.SetTargetPosition(ElevatorConstants::kFloorPickupPosition);
              break;
          }

          if(controller.GetBackButton()) elevator.SetTargetPosition(ElevatorConstants::kStartPosition);
      },
      {&elevator}));

      arm.SetDefaultCommand(frc2::RunCommand(
      [this] {
            arm.SetState(ArmConstants::kPositionMode);
            int pov = controller.GetPOV();
            switch(pov) {
            case 0: 
              arm.SetTargetPosition(ArmConstants::kHighDropoffPosition);
              break;
            case 90:
              arm.SetTargetPosition(ArmConstants::kMidDropoffPosition);
              break;
            case 180:
              arm.SetTargetPosition(ArmConstants::kFloorPickupPosition);
              break;
          }

          if(controller.GetBackButton()) arm.SetTargetPosition(ArmConstants::kStartPosition);
      },
      {&arm}));
}

void RobotContainer::ResetOdometry() {
        // m_drive.ResetOdometry({});
}

void RobotContainer::SetDriveBrakes(bool state) {
        m_drive.SetBrakeMode(state);
        elevator.SetBrakeMode(state);
        arm.SetBrakeMode(state);
        intake.SetBrakeMode(state);
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  // While holding the shoulder button, drive at half speed
//   frc2::JoystickButton(&controller, 5)
//       .WhenPressed(&m_driveHalfSpeed)
//       .WhenReleased(&m_driveFullSpeed);
  
//   frc2::JoystickButton(&controller, 1)
//       .WhenPressed(&toggleFlywheel);
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
      {frc::Pose2d{0.0_m, 0.0_m, 0.0_deg}, frc::Pose2d{1.0_m, 1.0_m, 0_deg}, 
      frc::Pose2d{-1.0_m, 2.0_m, 0_deg},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{0.0_m, 3.0_m, 0_deg},
      },
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController{AutoConstants::kPXController, 0, 0},
      frc2::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  // return std::move(swerveControllerCommand);
  return new frc2::SequentialCommandGroup(
      frc2::InstantCommand(
          [this]() { 
            m_drive.SetLimiting(false);
           }, {}),
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_deg_per_s, false);
            m_drive.SetLimiting(true);
          m_drive.SetInverted(false); 
          }, {}));
  
}