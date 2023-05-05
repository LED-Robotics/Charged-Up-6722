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

// return current Alliance from either FMS or Driver Station
bool RobotContainer::IsBlue() {
  return frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
}

// update SmartDashboard display of the currently selected station
void RobotContainer::UpdateStationUI() {
  frc::SmartDashboard::PutBoolean("station0", targetStation == 0);
  frc::SmartDashboard::PutBoolean("station1", targetStation == 1);
  frc::SmartDashboard::PutBoolean("station2", targetStation == 2);
  frc::SmartDashboard::PutBoolean("station3", targetStation == 3);
  frc::SmartDashboard::PutBoolean("station4", targetStation == 4);
  frc::SmartDashboard::PutBoolean("station5", targetStation == 5);
  frc::SmartDashboard::PutBoolean("station6", targetStation == 6);
  frc::SmartDashboard::PutBoolean("station7", targetStation == 7);
  frc::SmartDashboard::PutBoolean("station8", targetStation == 8);
  frc::SmartDashboard::PutBoolean("station9", targetStation == 9);
}

// iterate through each station position and find the closest target based on cone/cube mode
// if no position is within 3.0_m on the x axis this will return -1
int RobotContainer::GetClosestPosition() {
  bool coneMode = arm.GetConeMode();
  frc::Pose2d current = m_drive.GetPose();
  int pos = -1;
  double currentYDif = 1000.0;
  bool isBlue = IsBlue();
  for(int i = 0; i < 10; i++) {
  frc::Translation2d position = isBlue ? BlueHoldPositions[i] : RedHoldPositions[i];
    if(coneMode && (i == 1 || i == 4 || i == 7)) continue;
    else if(!coneMode && (i == 0 || i == 2 || i == 3 || i == 5 || i == 6 || i == 8)) continue;
    double deltaX = abs((double)current.X() - (double)position.X());
    if(deltaX > 3.0) continue;
    double deltaY = abs((double)current.Y() - (double)position.Y());
    if(deltaY < currentYDif) {
      pos = i;
      currentYDif = deltaY;
    }
  }
  return pos;
}

// utility function to return the appropriate SetPosition command
frc2::Command* RobotContainer::GetPositionCommand(int position) {
  return new SetPosition(position, &elevator, &arm, &intake);
}

// function passed to controller command bindings to allow different control schemes for solo/partner driving 
frc2::Command* RobotContainer::HandlePartnerCommands(frc2::Command* solo, frc2::Command* partner) {
  return new frc2::InstantCommand(
    [this, solo, partner]() { 
        if(controller2.IsConnected()) partner->Schedule();
        else solo->Schedule();
      }, {});
}

// returns dummy command
frc2::Command* RobotContainer::GetEmptyCommand() {
  return new frc2::InstantCommand(
          [this]() { 
           }, {});
}

// return a Pose2d object with the current station target pose based on user selection and current alliance
frc::Pose2d RobotContainer::GetTargetPose() {
  frc::Pose2d target{};
  bool isBlue = IsBlue();
  switch(targetStation) {
      case 0:
        return {isBlue ? BlueHoldPositions[0] : RedHoldPositions[0], {isBlue ? 180_deg : 0_deg}};
        break;
      case 1:
        return {isBlue ? BlueHoldPositions[1] : RedHoldPositions[1], {isBlue ? 180_deg : 0_deg}};
        break;
      case 2:
        return {isBlue ? BlueHoldPositions[2] : RedHoldPositions[2], {isBlue ? 180_deg : 0_deg}};
        break;
      case 3:
        return {isBlue ? BlueHoldPositions[3] : RedHoldPositions[3], {isBlue ? 180_deg : 0_deg}};
        break;
      case 4:
        return {isBlue ? BlueHoldPositions[4] : RedHoldPositions[4], {isBlue ? 180_deg : 0_deg}};
        break;
      case 5:
        return {isBlue ? BlueHoldPositions[5] : RedHoldPositions[5], {isBlue ? 180_deg : 0_deg}};
        break;
      case 6:
        return {isBlue ? BlueHoldPositions[6] : RedHoldPositions[6], {isBlue ? 180_deg : 0_deg}};
        break;
      case 7:
        return {isBlue ? BlueHoldPositions[7] : RedHoldPositions[7], {isBlue ? 180_deg : 0_deg}};
        break;
      case 8:
        return {isBlue ? BlueHoldPositions[8] : RedHoldPositions[8], {isBlue ? 180_deg : 0_deg}};
        break;
      case 9:
        return {isBlue ? BlueHoldPositions[9] : RedHoldPositions[9], {isBlue ? 90_deg : -90_deg}};
        break;
    }
    return target;
}

RobotContainer::RobotContainer() {

  // Autonomous selector configuration
  
  chooser.SetDefaultOption("High Dock", highDock.get());
  chooser.AddOption("Wall No Balance Blue", wallNoBalance.get());
  chooser.AddOption("Wall No Balance Red", wallNoBalanceRed.get());
  chooser.AddOption("Place Then Break", placeThenBreak.get());
  chooser.AddOption("Low Dock", lowDock.get());
  chooser.AddOption("Low Place Then Break", lowPlaceThenBreak.get());
  chooser.AddOption("Simple Dock", dock.get());
  chooser.AddOption("None", GetEmptyCommand());

  blinkin.Set(.41); // set LEDs to idle on startup when enabled

  SmartDashboard::PutData(&chooser);  // send auton selector to Shuffleboard

  odomTrigger.WhileTrue(&repeatOdom); // trigger to handle odom updates from AprilTags

  moveRoutineCancel.OnTrue(std::move(cancelMoves)); // run cancel command when driver interrupts auton subroutine in teleop

  // bindings for main driver elev, arm, intake positional control
  // main driver loses these controls if the partner is connected
  mainDpadUp.OnTrue(HandlePartnerCommands(GetPositionCommand(3), GetEmptyCommand()));
  mainDpadLeft.OnTrue(HandlePartnerCommands(GetPositionCommand(4), GetEmptyCommand()));
  mainDpadRight.OnTrue(HandlePartnerCommands(GetPositionCommand(2), GetEmptyCommand()));
  mainDpadDown.OnTrue(HandlePartnerCommands(GetPositionCommand(1), GetEmptyCommand()));
  controller.Back().OnTrue(HandlePartnerCommands(GetPositionCommand(0), GetEmptyCommand()));
  controller.Start().OnTrue(HandlePartnerCommands(GetPositionCommand(5), GetEmptyCommand()));

  // bindings for partner elev, arm, intake positional control
  partnerDpadUp.OnTrue(GetPositionCommand(3));
  partnerDpadLeft.OnTrue(GetPositionCommand(4));
  partnerDpadRight.OnTrue(GetPositionCommand(2));
  partnerDpadDown.OnTrue(GetPositionCommand(1));
  controller2.Y().OnTrue(GetPositionCommand(6));
  controller2.B().OnTrue(GetPositionCommand(0));

  // game element PUNCH binding! Very funny. 
  controller.LeftStick().OnTrue(std::move(punchObject));

  /* Triggers for auto-alignment selection Commands

  // controller.A().OnTrue(&togglePositionHold);
  // controller.B().OnTrue(&incrementStation);
  // controller.X().OnTrue(&decrementStation);
  // controller.RightStick().OnTrue(&setToSubstation);

  // Triggers for functional auto-alignment Commands
  
  // holdingTrigger.OnTrue(std::move(startHolding));
  // holdingTrigger.OnFalse(std::move(endHolding));
  // holdingTrigger.WhileTrue(std::move(holdPosition));
  */

  /* Triggers for enabling auto-docking Commands
  
  // controller.B().OnTrue(&toggleDocking);
  // controller.A().OnTrue(&toggleMaintain);
  // controller.B().OnTrue(&holdDocking);
  
  // Triggers for functional auto-docking Commands
  
  // dockTrigger.WhileTrue(std::move(teleDock));

  // maintainTrigger.OnFalse(endDocking);
  // maintainTrigger.WhileTrue(std::move(maintainEngage));
  */

  // partner LED control bindings
  controller2.A().OnTrue(&SetBlinkinAButton);
  controller2.X().ToggleOnTrue(&verticalPickup);
  controller2.LeftBumper().OnTrue(&SetBlinkinLeftBumper);
  controller2.RightBumper().OnTrue(&SetBlinkinRightBumper);

  // funny rumble command bindings. These might not work. 
  controller.Start().WhileTrue(&rumbleSecondaryOn);
  controller2.Start().WhileTrue(&rumblePrimaryOn);
  controller.Start().OnFalse(&rumbleSecondaryOff);
  controller2.Start().OnFalse(&rumblePrimaryOff);

  // Set up default drive command
    m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
            // lazy non-command implementation of a field-centric drive toggle
            if(controller.GetYButtonPressed()) fieldCentric = !fieldCentric;

            // store control inputs for driving
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
            // pass filtered inputs to Drive function
            // inputs will be between -1.0 to 1.0, multiply by intended speed range in mps/deg_per_s when passing
            m_drive.Drive(
              units::meters_per_second_t{xSpeed * -3.5},
              units::meters_per_second_t{ySpeed * -3.5},
              
              units::degrees_per_second_t{turn * 226.0}, fieldCentric);
      },
      {&m_drive}));
    
    // function to handle intake logic. Should probably be a command
    intake.SetDefaultCommand(frc2::RunCommand(
      [this] {
          HandleIntake();
      },
      {&intake}));

    // Ensure elevator is running in position mode during normal operation
    elevator.SetDefaultCommand(frc2::RunCommand(
      [this] {
            elevator.SetState(ElevatorConstants::kPositionMode);
      },
      {&elevator}));

    // Ensure arm is running in position mode during normal operation
      arm.SetDefaultCommand(frc2::RunCommand(
      [this] {
            arm.SetState(ArmConstants::kPositionMode);
      },
      {&arm}));

      // command to trigger odom updates from limelight AprilTag data
      armLimelight.SetDefaultCommand(frc2::RunCommand(
      [this] {
        if(!armLimelight.IsTarget() || armLimelight.GetTargetArea() < 0.65) {
          validTag = false; // if there is no tag detected or the tag is very small don't update odom!
        } else {
          validTag = true;  // positional data most likely good. Update odom! 
        }
          frc::SmartDashboard::PutBoolean("tagDetected", validTag); // flashes green to tell drivers odom has been updated
      },
      {&armLimelight}));


}

void RobotContainer::EnableTagTracking() {
  tagOverrideDisable = false;
}

void RobotContainer::DisableTagTracking() {
  tagOverrideDisable = true;
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

// return selected auton routine from Shuffleboard
frc2::Command* RobotContainer::GetAutonomousCommand() {
    auto selected = chooser.GetSelected();
    return selected;
}