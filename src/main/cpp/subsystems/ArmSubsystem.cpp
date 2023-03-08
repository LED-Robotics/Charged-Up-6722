// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

using namespace ArmConstants;
using namespace frc;

ArmSubsystem::ArmSubsystem()
    : left{kLeftMotorPort},
    right{kRightMotorPort} {
      left.SetInverted(true);
      left.SetSelectedSensorPosition(0);
      right.SetSelectedSensorPosition(0);
}

void ArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // std::cout << "Left Arm Current Position: " << GetLeftPosition() << '\n';
  // std::cout << "Right Arm Current Position: " << GetRightPosition() << '\n';
  // std::cout << "Left Velocity: " << left.GetSelectedSensorVelocity() << '\n';
  if(state == kOff) {
    left.Set(0.0);
    right.Set(0.0);
  } else if(state == kPowerMode) {
    left.Set(power);
    right.Set(power);
  } else if(state == kPositionMode) {
    // std::cout << "Arm Position mode: " << position << '\n';
    // feed forwards should be a changing constant that increases as the arm moves further. It should be a static amount of power to overcome gravity.
    // double leftFeedForward = 0.0; // GetLeftPosition() / 5000   <-- tune this number after verifying the motion magic works in any capacity
    double leftFeedForward = sin((GetLeftPosition() / kCountsPerDegree) * (M_PI/180)) * kMaxFeedForward;
    // std::cout << "Left Feed Forward: " << leftFeedForward << '\n';
    // leftFeedForward = sin(currentAngle) * kF;

    left.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position, position, ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, leftFeedForward);
    // double rightFeedForward = 0.0; // GetRightPosition() / 5000   <-- tune this number after verifying the motion magic works in any capacity
    double rightFeedForward = sin((GetRightPosition() / kCountsPerDegree) * (M_PI/180)) * kMaxFeedForward;
    right.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position, position, ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, rightFeedForward);
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