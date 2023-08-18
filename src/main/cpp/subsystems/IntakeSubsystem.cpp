// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace IntakeConstants;
using namespace frc;

IntakeSubsystem::IntakeSubsystem(ArmSubsystem *reference)
    : intakeMotor{kIntakePort, "canUknot"},
    wristMotor{kWristPort, "canUknot"} {
      arm = reference;
      intakeMotor.SetInverted(true);
      // wristMotor.SetSelectedSensorPosition(0);
      // ConfigMotors();
}

void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Intake power control
  if(state == kOff) {
    intakeMotor.Set(0.0);
  } else if(state == kPowerMode) {
    // if(intakeMotor.GetOutputCurrent() < kCurrentLimit && power > 0.0) intakeMotor.Set(power);
    intakeMotor.Set(power);
    // else intakeMotor.Set(0.0);
  }

  // Wrist position control
  // double armAngle = arm->GetAngle();
  // std::cout << "Arm Angle: " << armAngle << '\n';
  // std::cout << "Wrist Pos: " << wristMotor.GetSelectedSensorPosition() << '\n';
  // feed forward should be a changing constant that increases as the wrist moves further. It should be a static amount of power to overcome gravity.
  double wristAngle = GetCurrentAngle() * (M_PI/180);
  // double feedForward = 0.0;
  double feedForward = sin(wristAngle) * kMaxFeedForward;
  // SmartDashboard::PutNumber("wristAngle", GetCurrentAngle());  // print to Shuffleboard
  // SmartDashboard::PutNumber("wristPos", wristMotor.GetSelectedSensorPosition());  // print to Shuffleboard
  if(wristState == kPositionMode) {
    wristMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, position, ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, feedForward);
  } else if(wristState == kAngleMode) {
    wristMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, (angle * kCountsPerDegree) + (arm->GetAngle() * kCountsPerDegree), ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, feedForward);

  }
}

void IntakeSubsystem::Off() {
  state = kOff;
}

void IntakeSubsystem::On() {
  state = kFullMode;
}

void IntakeSubsystem::UsePowerMode() {
  state = kPowerMode;
}

void IntakeSubsystem::SetPower(double newPower) {
  power = newPower;
  // if(power < kIntakeDeadzone) power = 0.0;
}

void IntakeSubsystem::SetWristPower(double newPower) {
  wristPower = newPower;
  // if(power < kIntakeDeadzone) power = 0.0;
}

double IntakeSubsystem::GetPower() {
  return power;
}

int IntakeSubsystem::GetState() {
  return state;
}

void IntakeSubsystem::SetState(int newState) {
  state = newState;
}

int IntakeSubsystem::GetWristState() {
  return wristState;
}

void IntakeSubsystem::SetWristState(int newState) {
  wristState = newState;
}

void IntakeSubsystem::SetTargetPosition(double newPosition) {
  position = newPosition;
}

double IntakeSubsystem::GetTargetPosition() {
  return position;
}

double IntakeSubsystem::GetCurrentPosition() {
  return wristMotor.GetSelectedSensorPosition();
}

void IntakeSubsystem::SetTargetAngle(double newAngle) {
  angle = newAngle;
}

double IntakeSubsystem::GetTargetAngle() {
  return angle;
}

double IntakeSubsystem::GetCurrentAngle() {
  return kStartAngle - arm->GetAngle() + (GetCurrentPosition() / kCountsPerDegree);
  
}

void IntakeSubsystem::ResetWristEncoder() {
  wristMotor.SetSelectedSensorPosition(0);
}

bool IntakeSubsystem::IsAtTarget() {
  double current = 0.0;
  if(wristState == kPositionMode) {
    current = GetCurrentPosition();
  } else if(wristState == kAngleMode) {
    current = GetCurrentAngle();
  }
  bool atTarget = current > position - (kPositionDeadzone / 2) && current < position + (kPositionDeadzone / 2);
  return atTarget;
}

void IntakeSubsystem::SetBrakeMode(bool state) {
  ctre::phoenix::motorcontrol::NeutralMode mode;
  if(state) mode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
  else mode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
  intakeMotor.SetNeutralMode(mode);
  wristMotor.SetNeutralMode(mode);
}

void IntakeSubsystem::ConfigMotors() {
  wristMotor.Config_kP(0, kP, 100);
}