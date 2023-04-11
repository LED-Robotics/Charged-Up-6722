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

frc2::Command* RobotContainer::GetPositionCommand(int position) {
  return new SetPosition(position, &elevator, &arm, &intake);
}

frc2::Command* RobotContainer::GetRelativePathCommand(const Pose2d& start, const std::vector<Translation2d>& interiorWaypoints,
    const Pose2d& end, const TrajectoryConfig& config) {
  return new TrajectoryRelative(start, interiorWaypoints, end, config, &m_drive);
}

frc2::Command* RobotContainer::HandlePartnerCommands(frc2::Command* solo, frc2::Command* partner) {
  return new frc2::InstantCommand(
    [this, solo, partner]() { 
        if(controller2.IsConnected()) partner->Schedule();
        else solo->Schedule();
      }, {});
}

frc2::Command* RobotContainer::GetEmptyCommand() {
  return new frc2::InstantCommand(
          [this]() { 
           }, {});
}

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  chooser.SetDefaultOption("Low Dock", lowDock.get());
  chooser.AddOption("High Dock", highDock.get());
  chooser.AddOption("Simple Dock", dock.get());
  chooser.AddOption("Low Place Then Break", lowPlaceThenBreak.get());
  chooser.AddOption("Place Then Break", placeThenBreak.get());
  chooser.AddOption("Wall No Balance", wallNoBalance.get());
  chooser.AddOption("None", GetEmptyCommand());

  blinkin.Set(.41);

  SmartDashboard::PutData(&chooser);

  odomTrigger.WhileTrue(&repeatOdom);

  alignCancelTrigger.OnTrue(frc2::cmd::RunOnce([this] { 
    stationAlignCancel = true;
  }, {})
  ).OnFalse(frc2::cmd::RunOnce([this] { 
    stationAlignCancel = false;
  }, {}));

  mainDpadUp.OnTrue(HandlePartnerCommands(GetPositionCommand(3), GetEmptyCommand()));
  mainDpadLeft.OnTrue(HandlePartnerCommands(GetPositionCommand(4), GetEmptyCommand()));
  mainDpadRight.OnTrue(HandlePartnerCommands(GetPositionCommand(2), GetEmptyCommand()));
  mainDpadDown.OnTrue(HandlePartnerCommands(GetPositionCommand(1), GetEmptyCommand()));
  controller.Back().OnTrue(HandlePartnerCommands(GetPositionCommand(0), GetEmptyCommand()));
  controller.Start().OnTrue(HandlePartnerCommands(GetPositionCommand(5), GetEmptyCommand()));
  // controller.B().ToggleOnTrue(GetRelativePathCommand({0.0_m, 0.0_m, 0_deg}, {}, {-4.0_m, -0.05_m, 0_deg}, {AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}));
  // controller.X().ToggleOnTrue(&turnTo90);
  // controller.A().ToggleOnTrue(GetRelativePathCommand({0.0_m, 0.0_m, 0_deg}, {}, {4.0_m, -0.0_m, 0_deg}, {AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}));

  // controller.B().ToggleOnTrue(std::move(testAuto));
  // controller.B().ToggleOnTrue(&driveRateTest);
  // controller.X().ToggleOnTrue(std::move(testPath));
  // controller.A().ToggleOnTrue(std::move(testRotate));
  controller.A().OnTrue(&toggleStationAlign);
  controller.B().OnTrue(&decrementStation);
  controller.X().OnTrue(&incrementStation);

  alignTrigger.OnTrue(std::move(alignFollow));

  // controller.A().ToggleOnTrue(&driveL);
  // controller.X().ToggleOnTrue(&driveL2);
  // controller.B().ToggleOnTrue(new GyroDock(1.5, &m_drive));

  partnerDpadUp.OnTrue(GetPositionCommand(3));
  partnerDpadLeft.OnTrue(GetPositionCommand(4));
  partnerDpadRight.OnTrue(GetPositionCommand(2));
  partnerDpadDown.OnTrue(GetPositionCommand(1));
  controller2.B().OnTrue(GetPositionCommand(0));
  controller2.A().OnTrue(&SetBlinkinAButton);
  controller2.X().ToggleOnTrue(&verticalPickup);
  controller2.LeftBumper().OnTrue(&SetBlinkinLeftBumper);
  controller2.RightBumper().OnTrue(&SetBlinkinRightBumper);

  controller.Start().OnTrue(&rumbleSecondaryOn);
  controller2.Start().OnTrue(&rumblePrimaryOn);
  controller.Start().OnFalse(&rumbleSecondaryOff);
  controller2.Start().OnFalse(&rumblePrimaryOff);

  // Set up default drive command
    m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
            field.SetRobotPose(m_drive.GetPose());
            if(controller.GetYButtonPressed()) fieldCentric = !fieldCentric;
            double x = controller.GetLeftY();
            double y = controller.GetLeftX();
            double turnX = controller.GetRightX();

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
            float turn = 0.95 * pow(turnX, 3) + (1 - 0.95) * turnX;
            m_drive.Drive(
              units::meters_per_second_t{xSpeed * -3.5},
              units::meters_per_second_t{ySpeed * -3.5},
              
              units::degrees_per_second_t{turn * 226.0}, fieldCentric);
      },
      {&m_drive}));
    
    intake.SetDefaultCommand(frc2::RunCommand(
      [this] {
          HandleIntake();
      },
      {&intake}));

    elevator.SetDefaultCommand(frc2::RunCommand(
      [this] {
            elevator.SetState(ElevatorConstants::kPositionMode);
      },
      {&elevator}));

      arm.SetDefaultCommand(frc2::RunCommand(
      [this] {
            arm.SetState(ArmConstants::kPositionMode);
      },
      {&arm}));

      armLimelight.SetDefaultCommand(frc2::RunCommand(
      [this] {
        frc::SmartDashboard::PutData("field", &field);
        if(!armLimelight.IsTarget() || armLimelight.GetTargetArea() < 0.65) {
          validTag = false;
        } else {
          validTag = true;
        }
          frc::SmartDashboard::PutBoolean("tagDetected", validTag);
      },
      {&armLimelight}));


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

void RobotContainer::HandleIntake() {
  intake.SetState(IntakeConstants::kPowerMode);
  double speed = controller.GetRightTriggerAxis() - controller.GetLeftTriggerAxis();
  if(speed > 0.1 || speed < -0.1) {
    intakeHold = false;
    intake.SetPower(speed);
  } else if(!intakeHold) intake.SetPower(0.0);
  if(controller.GetRightBumperPressed()) {
    intakeHold = true;
    intake.SetPower(0.1);
  }
  if(controller.GetLeftBumperPressed()) {
    intake.SetPower(0.0);
  }
}

void RobotContainer::SetSlew(bool state) {
  m_drive.SetLimiting(state);
}

void RobotContainer::SetDriveReversed(bool reversed) {
  m_drive.SetInverted(reversed);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    auto selected = chooser.GetSelected();
    if(selected->GetName() == dock.get()->GetName()) {
      // flip odometry so that field centric works correctly
      startOffset = 0_deg;
      m_drive.ResetOdometry(frc::Pose2d{{0.0_m, 0.0_m}, {0_deg}});
    }
    return selected;
}