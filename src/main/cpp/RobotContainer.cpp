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

  // Configure the button bindings
  ConfigureButtonBindings();

  chooser.SetDefaultOption("High Dock", &highDock);
  chooser.AddOption("Simple Dock", &dock);
  chooser.AddOption("None", GetEmptyCommand());

  SmartDashboard::PutData(&chooser);

  // dpadUp.OnTrue(GetPositionCommand(3));
  // dpadRight.OnTrue(GetPositionCommand(2));
  // dpadDown.OnTrue(GetPositionCommand(1));
  // controller.Back().OnTrue(GetPositionCommand(0));

  mainDpadUp.OnTrue(HandlePartnerCommands(GetPositionCommand(3), GetEmptyCommand()));
  mainDpadLeft.OnTrue(HandlePartnerCommands(GetPositionCommand(4), GetEmptyCommand()));
  mainDpadRight.OnTrue(HandlePartnerCommands(GetPositionCommand(2), GetEmptyCommand()));
  mainDpadDown.OnTrue(HandlePartnerCommands(GetPositionCommand(1), GetEmptyCommand()));
  controller.Back().OnTrue(HandlePartnerCommands(GetPositionCommand(0), GetEmptyCommand()));
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
            if(controller.GetYButtonPressed()) fieldCentric = !fieldCentric;
            double x = controller.GetLeftX();
            double y = controller.GetLeftY();
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
              units::meters_per_second_t{ySpeed * 3.5},
              units::meters_per_second_t{xSpeed * -3.5},
              units::degrees_per_second_t{turn * 130.0}, fieldCentric);
      },
      {&m_drive}));
    
    intake.SetDefaultCommand(frc2::RunCommand(
      [this] {
          HandleIntake();
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
          //   int pov = controller.GetPOV();
          //   switch(pov) {
          //   case 0: 
          //     elevator.SetTargetPosition(ElevatorConstants::kHighDropoffPosition);
          //     break;
          //   case 90:
          //     elevator.SetTargetPosition(ElevatorConstants::kMidDropoffPosition);
          //     break;
          //   case 180:
          //     elevator.SetTargetPosition(ElevatorConstants::kFloorPickupPosition);
          //     break;
          // }

          // if(controller.GetBackButton()) elevator.SetTargetPosition(ElevatorConstants::kStartPosition);
      },
      {&elevator}));

      arm.SetDefaultCommand(frc2::RunCommand(
      [this] {
            arm.SetState(ArmConstants::kPositionMode);
          //   int pov = controller.GetPOV();
          //   switch(pov) {
          //   case 0: 
          //     arm.SetTargetPosition(ArmConstants::kHighDropoffPosition);
          //     break;
          //   case 90:
          //     arm.SetTargetPosition(ArmConstants::kMidDropoffPosition);
          //     break;
          //   case 180:
          //     arm.SetTargetPosition(ArmConstants::kFloorPickupPosition);
          //     break;
          // }

          // if(controller.GetBackButton()) arm.SetTargetPosition(ArmConstants::kStartPosition);
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
  // Set up config for trajectory
  
  // frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
  //                              AutoConstants::kMaxAcceleration);
  // // Add kinematics to ensure max speed is actually obeyed
  // config.SetKinematics(m_drive.kDriveKinematics);

  // // An example trajectory to follow.  All units in meters.
  // auto moveFromLink = frc::TrajectoryGenerator::GenerateTrajectory(
    
  //     // Start at the origin facing the +X direction
  //     {frc::Pose2d{0.0_m, 0.0_m, 0.0_deg},
  //     // End 3 meters straight ahead of where we started, facing forward
  //     frc::Pose2d{-0.5_m, 0.0_m, 0.0_deg},
  //     },
  //     // Pass the config
  //     config);

  // auto moveToLink = frc::TrajectoryGenerator::GenerateTrajectory(
    
  //     // Start at the origin facing the +X direction
  //     {frc::Pose2d{-0.5_m, 0.0_m, 0.0_deg},
  //     // End 3 meters straight ahead of where we started, facing forward
  //     frc::Pose2d{0.0_m, 0.0_m, 0.0_deg},
  //     },
  //     // Pass the config
  //     config);

  // frc::ProfiledPIDController<units::radians> thetaController{
  //     AutoConstants::kPThetaController, 0, 0,
  //     AutoConstants::kThetaControllerConstraints};

  // thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
  //                                       units::radian_t{std::numbers::pi});
  // frc2::SwerveControllerCommand<4> commandMoveFromLink(
  //     moveFromLink, [this]() { return m_drive.GetPose(); },

  //     m_drive.kDriveKinematics,

  //     frc2::PIDController{AutoConstants::kPXController, 0, 0},
  //     frc2::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

  //     [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

  //     {&m_drive});

  // frc2::SwerveControllerCommand<4> commandMoveToLink(
  //     moveToLink, [this]() { return m_drive.GetPose(); },

  //     m_drive.kDriveKinematics,

  //     frc2::PIDController{AutoConstants::kPXController, 0, 0},
  //     frc2::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

  //     [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

  //     {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  // m_drive.ResetOdometry(moveFromLink.InitialPose());

  // return new frc2::SequentialCommandGroup(
  //     HighDock(&m_drive, &elevator, &arm, &intake),
  //     frc2::InstantCommand(
  //         [this]() { m_drive.Drive(0_mps, 0_mps, 0_deg_per_s, false);
  //         }, {}));
  // return new frc2::SequentialCommandGroup(
  //     GyroDock(1.5, &m_drive),
  //     frc2::InstantCommand(
  //         [this]() { m_drive.Drive(0_mps, 0_mps, 0_deg_per_s, false);
  //         }, {}));
    // return new HighDock(&m_drive, &elevator, &arm, &intake);
    frc2::Command* selected = chooser.GetSelected();
    if(selected == &highDock) {
      // flip odometry so that field centric works correctly
      m_drive.ResetOdometry(frc::Pose2d{{0.0_m, 0.0_m}, {180_deg}});
    }
    return selected;
}