// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <iostream>
#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>
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
            // m_drive.ArcadeDrive(controller.GetLeftY(),
            //                 controller.GetLeftX() * -1);
            m_drive.ArcadeDrive(controller.GetRightTriggerAxis() - controller.GetLeftTriggerAxis(),
                            controller.GetLeftX());
      },
      {&m_drive}));

    flywheel.SetDefaultCommand(frc2::RunCommand(
      [this] {
          if(controller.GetAButtonPressed()) flywheel.SetFlywheelState(!flywheel.GetFlywheelState());
          if(controller.GetYButtonPressed()) flywheel.SetMotorPower(flywheel.GetMotorPower() + 0.01);
          if(controller.GetBButtonPressed()) flywheel.SetMotorPower(flywheel.GetMotorPower() - 0.01);
      },
      {&flywheel}));
    
    intake.SetDefaultCommand(frc2::RunCommand(
      [this] {
            intake.SetPower(controller2.GetRightTriggerAxis() - controller2.GetLeftTriggerAxis());
            if(controller.GetXButton()) {
                intake.On();
            } else {
                if(intake.GetPower() > IntakeConstants::kIntakeDeadzone) {
                    intake.UsePowerMode();
                } else intake.Off();
            }
      },
      {&intake}));

    elevator.SetDefaultCommand(frc2::RunCommand(
      [this] {
            int pov = controller.GetPOV();
            if(pov != 0 && pov != 180) pov = controller2.GetPOV();
            if(pov == 0) {
                elevator.On();
                elevator.SetPower(ElevatorConstants::kDefaultPower);
            } else if(pov == 180) {
                elevator.On();
                elevator.SetPower(-ElevatorConstants::kDefaultPower);
            } else {
                elevator.Off();
            }
      },
      {&elevator}));

    turret.SetDefaultCommand(frc2::RunCommand(
      [this] {
            int pov = controller.GetPOV();
            if(pov != 90 && pov != 270) pov = controller2.GetPOV();
            if(pov == 90) {
                turret.SetState(TurretConstants::kPowerMode);
                turret.SetPower(TurretConstants::kDefaultPower);
            } else if(pov == 270) {
                turret.SetState(TurretConstants::kPowerMode);
                turret.SetPower(-TurretConstants::kDefaultPower);
            } else {
                turret.SetState(TurretConstants::kOff);
            }
      },
      {&turret}));

      lift.SetDefaultCommand(frc2::RunCommand(
      [this] {
          lift.SetState(LiftConstants::kPowerMode);
          lift.SetPower(abs(controller.GetRightY()) > 0.1 ? controller.GetRightY() : 0);
      },
      {&lift}));
}

void RobotContainer::ResetOdometry() {
        m_drive.ResetOdometry({});
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  // While holding the shoulder button, drive at half speed
  frc2::JoystickButton(&controller, 5)
      .WhenPressed(&m_driveHalfSpeed)
      .WhenReleased(&m_driveFullSpeed);
  
//   frc2::JoystickButton(&controller, 1)
//       .WhenPressed(&toggleFlywheel);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Create a voltage constraint to ensure we don't accelerate too fast
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics, 10_V);

  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(DriveConstants::kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(0_m, 1_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(0_m, 1_m, frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  frc2::RamseteCommand ramseteCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },
      frc::RamseteController(AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
}
