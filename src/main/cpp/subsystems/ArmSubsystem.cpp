// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ArmConstants;
using namespace frc;

ArmSubsystem::ArmSubsystem()
    : left{kLeftMotorPort},
    right{kRightMotorPort} {
      left.SetInverted(true);
      // left.SetSelectedSensorPosition(0);
      // right.SetSelectedSensorPosition(0);
      // ConfigMotors();
}

void ArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  if(state == kOff) {
    left.Set(0.0);
    right.Set(0.0);
  } else if(state == kPowerMode) {
    left.Set(power);
    right.Set(power);
  } else if(state == kPositionMode) {
    // std::cout << "Arm Position mode: " << position << '\n';
    // feed forwards should be a changing constant that increases as the arm moves further. It should be a static amount of power to overcome gravity.
    
    // double leftAngle = (GetLeftPosition() / kCountsPerDegree) * (M_PI/180);
    // double rightAngle = (GetRightPosition() / kCountsPerDegree) * (M_PI/180);
    // double leftFeedForward = sin(leftAngle) * kMaxFeedForward;
    // double rightFeedForward = sin(rightAngle) * kMaxFeedForward;
    // SmartDashboard::PutNumber("armAngle", ((GetLeftPosition() / kCountsPerDegree) + (GetRightPosition() / kCountsPerDegree)) / 2);  // print to Shuffleboard
    // left.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position, position, ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, leftFeedForward);
    // right.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position, position, ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, rightFeedForward);

    double leftAngle = (GetLeftPosition() / kCountsPerDegree) * (M_PI/180);
    double rightAngle = (GetRightPosition() / kCountsPerDegree) * (M_PI/180);
    double leftFeedForward = sin(leftAngle) * kMaxFeedForward;
    double rightFeedForward = sin(rightAngle) * kMaxFeedForward;
    SmartDashboard::PutBoolean("coneMode", GetConeMode());  // print to Shuffleboard
    // SmartDashboard::PutNumber("armAngle", ((GetLeftPosition() / kCountsPerDegree) + (GetRightPosition() / kCountsPerDegree)) / 2);  // print to Shuffleboard
    left.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position, angle * kCountsPerDegree, ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, leftFeedForward);
    right.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position, angle * kCountsPerDegree, ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, rightFeedForward);
  }
}

void ArmSubsystem::Off() {
  state = kOff;
}

void ArmSubsystem::On() {
  state = kPowerMode;
}

void ArmSubsystem::SetPower(double newPower) {
  power = newPower;
}

void ArmSubsystem::SetState(int newState) {
  state = newState;
}

int ArmSubsystem::GetState() {
  return state;
}

void ArmSubsystem::SetTargetPosition(double newPosition) {
  position = newPosition;
}

double ArmSubsystem::GetLeftPosition() {
  return left.GetSelectedSensorPosition(0);
}

double ArmSubsystem::GetRightPosition() {
  return left.GetSelectedSensorPosition(0);
}

double ArmSubsystem::GetAngle() {
  double left = GetLeftPosition() / kCountsPerDegree;
  double right = GetRightPosition() / kCountsPerDegree;
  return (left + right) / 2;
}

void ArmSubsystem::SetTargetAngle(double newAngle) {
  angle = newAngle;
}

bool ArmSubsystem::IsAtTarget() {
  double target = angle * kCountsPerDegree;
  double leftPos = GetLeftPosition();
  double rightPos = GetRightPosition();
  bool leftAtTarget = leftPos > target - (kPositionDeadzone / 2) && leftPos < target + (kPositionDeadzone / 2);
  bool rightAtTarget = rightPos > target - (kPositionDeadzone / 2) && rightPos < target + (kPositionDeadzone / 2);
  return leftAtTarget && rightAtTarget;
}

void ArmSubsystem::SetBrakeMode(bool state) {
  ctre::phoenix::motorcontrol::NeutralMode mode;
  if(state) mode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
  else mode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
  left.SetNeutralMode(mode);
  right.SetNeutralMode(mode);
}

void ArmSubsystem::SetConeMode() {
  isCone = true;
}

void ArmSubsystem::SetCubeMode() {
  isCone = false;
}

bool ArmSubsystem::GetConeMode() {
  return isCone;
}

void ArmSubsystem::ConfigMotors() {
  left.Config_kP(0, kP, 100);
  right.Config_kP(0, kP, 100);
}