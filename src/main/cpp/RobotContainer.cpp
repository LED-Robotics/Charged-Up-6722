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
            if(controller.GetAButton()) {
              if(m_drive.ZeroSwervePosition()) {
                m_drive.ResetEncoders();
              }
            } else {
              m_drive.Drive(
                units::meters_per_second_t{controller.GetLeftY() * 4.0},
                units::meters_per_second_t{controller.GetLeftX() * -4.0},
                units::degrees_per_second_t{controller.GetRightX() * 150}, false);
            }
            // D-Pad Turn Control
            // double angle = controller.GetPOV();
            // if(angle != -1) {
            //     frc::SwerveModuleState state{0_mps, {units::degree_t{angle}}};
            //     m_drive.SetModuleStates({state, state, state, state});
            // }
            // double axis = controller.GetLeftY();
            // auto speed = 1.5_mps;
            // if(abs(axis) > 0.3) {
            //   frc::SwerveModuleState state{axis > 0.0 ? speed : -speed, {units::degree_t{0}}};
            //   m_drive.SetModuleStates({state, state, state, state});
            // } else {
            //   frc::SwerveModuleState state{0_mps, {units::degree_t{0}}};
            //   m_drive.SetModuleStates({state, state, state, state});
            // }
            // m_drive.SetDrivePower(controller.GetLeftY());
            // Manual turn control
            // m_drive.SetTurnPower(controller.GetRightX());
      },
      {&m_drive}));
    
    intake.SetDefaultCommand(frc2::RunCommand(
      [this] {
          // intake.SetState(IntakeConstants::kPowerMode);
          // if(controller.GetXButton()) intake.SetPower(1.0);
          // else if(controller.GetYButton()) intake.SetPower(-1.0);
          // else intake.SetPower(0.0);
          intake.SetPosition(4000 + (controller.GetRightTriggerAxis() * 16600));

          // intake.SetState(ArmConstants::kPowerMode);
          // double wristPower = SmartDashboard::GetNumber("wristPower", 0.0);
          // intake.SetWristPower(wristPower);
      },
      {&intake}));

    // elevator.SetDefaultCommand(frc2::RunCommand(
    //   [this] {
    //         elevator.SetTargetPosition(controller.GetLeftTriggerAxis() * 30000);
    //         int state = elevator.GetState();
    //         if(controller.GetAButtonPressed()) {
    //           if(state == ElevatorConstants::kPositionMode) elevator.SetState(ElevatorConstants::kOff);
    //           else if(state == ElevatorConstants::kOff) elevator.SetState(ElevatorConstants::kPositionMode);
    //         }
    //   },
    //   {&elevator}));

      arm.SetDefaultCommand(frc2::RunCommand(
      [this] {
            // arm.SetTargetPosition(controller.GetRightTriggerAxis() * 80000);
            // int state = arm.GetState();
            // if(controller.GetBButtonPressed()) {
            //   if(state == ArmConstants::kPositionMode) arm.SetState(ArmConstants::kOff);
            //   else if(state == ArmConstants::kOff) arm.SetState(ArmConstants::kPositionMode);
            // }

            // arm.SetState(ArmConstants::kPositionMode);
            // double armPos = SmartDashboard::GetNumber("armPos", 6435);
            // arm.SetTargetPosition(armPos);

            arm.SetState(ArmConstants::kPositionMode);
            arm.SetTargetPosition(6435 + (controller.GetLeftTriggerAxis() * 80000));

            // arm.SetState(ArmConstants::kPowerMode);
            // double armPower = SmartDashboard::GetNumber("armPower", 0.0);
            // arm.SetPower(armPower);
      },
      {&arm}));

    // turret.SetDefaultCommand(frc2::RunCommand(
    //   [this] {
    //         int pov = controller.GetPOV();
    //         if(pov != 90 && pov != 270) pov = controller2.GetPOV();
    //         if(pov == 90) {
    //             turret.SetState(TurretConstants::kPowerMode);
    //             turret.SetPower(TurretConstants::kDefaultPower);
    //         } else if(pov == 270) {
    //             turret.SetState(TurretConstants::kPowerMode);
    //             turret.SetPower(-TurretConstants::kDefaultPower);
    //         } else {
    //             turret.SetState(TurretConstants::kOff);
    //         }
    //   },
    //   {&turret}));

    //   lift.SetDefaultCommand(frc2::RunCommand(
    //   [this] {
    //       lift.SetState(LiftConstants::kPowerMode);
    //       lift.SetPower(abs(controller.GetRightY()) > 0.1 ? controller.GetRightY() : 0);
    //   },
    //   {&lift}));
}

void RobotContainer::ResetOdometry() {
        // m_drive.ResetOdometry({});
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
  /*
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    /*
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
            m_drive.SetInverted(true);
           }, {}),
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false);
          m_drive.SetInverted(false); }, {}));
  */
}

frc2::Command* RobotContainer::GetZeroCommand() {
}