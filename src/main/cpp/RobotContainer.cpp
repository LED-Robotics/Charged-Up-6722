// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>

// return current Alliance from either FMS or Driver Station
bool RobotContainer::IsBlue() {
  return frc::DriverStation::GetAlliance() ==
         frc::DriverStation::Alliance::kBlue;
}
// update SmartDashboard display of the currently selected station

RobotContainer::RobotContainer() {

  drive.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        // store control inputs for driving
        double x = -controller.GetLeftY();
        double y = -controller.GetLeftX();
        double turnX = -controller.GetRightX();

        // zero out axes if they fall within deadzon
        if (x > -DriveConstants::kDriveDeadzone &&
            x < DriveConstants::kDriveDeadzone)
          x = 0.0;
        if (y > -DriveConstants::kDriveDeadzone &&
            y < DriveConstants::kDriveDeadzone)
          y = 0.0;

        if (turnX > -DriveConstants::kTurnDeadzone &&
            turnX < DriveConstants::kTurnDeadzone)
          turnX = 0.0;

        // put speeds through a polynomial to smooth out joystick input
        // check the curve out here:
        // https://www.desmos.com/calculator/65tpwhxyai the range between 0.0
        // to 1.0 is used for the motors change driveCurveExtent to modify curve
        // strength
        float xSpeed = DriveConstants::kDriveCurveExtent * pow(x, 3) +
                       (1 - DriveConstants::kDriveCurveExtent) * x;
        float ySpeed = DriveConstants::kDriveCurveExtent * pow(y, 3) +
                       (1 - DriveConstants::kDriveCurveExtent) * y;
        float turn = 0.3 * pow(turnX, 3) + (1 - 0.3) * turnX;
        // pass filtered inputs to Drive function
        // inputs will be between -1.0 to 1.0, multiply by intended speed range
        // in mps/deg_per_s when passing

        drive.Drive(
            {units::velocity::meters_per_second_t{
                 xSpeed * DriveConstants::kDriveTranslationLimit.value()},
             units::velocity::meters_per_second_t{
                 ySpeed * DriveConstants::kDriveTranslationLimit.value()},
             turn * -270.0_deg_per_s},
            true, fieldCentric);
      },
      {&drive}));

  telescope.SetDefaultCommand(frc2::cmd::Run(
      [this] {

      },
      {&telescope}));

  arm.SetDefaultCommand(frc2::cmd::Run(
      [this] {

      },
      {&arm}));

  wrist.SetDefaultCommand(frc2::cmd::Run(
      [this] {

      },
      {&wrist}));

  intake.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        intake.SetState(IntakeConstants::kPowerMode);
        double speed =
            controller.GetRightTriggerAxis() - controller.GetLeftTriggerAxis();
        if (speed > 0.1 || speed < -0.1) {
          intakeHold = false;
          intake.SetPower(speed);
        } else if (!intakeHold)
          intake.SetPower(0.0);
        if (controller.GetHID().GetRightBumperButtonPressed()) {
          intakeHold = true;
          intake.SetPower(0.1);
        }
      },
      {&intake}));
}
