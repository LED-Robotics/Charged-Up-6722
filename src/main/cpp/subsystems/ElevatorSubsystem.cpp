// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ElevatorConstants;
using namespace frc;

ElevatorSubsystem::ElevatorSubsystem()
    : left{kLeftMotorPort},
    right{kRightMotorPort} {
      right.SetInverted(true);
}

void ElevatorSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  if(state == kOff) {
    left.Set(0.0);
    right.Set(0.0);
  } else if(state == kPowerMode) {
    left.Set(power);
    right.Set(power);
  } else if(state == kPositionMode) {
      double leftPos = GetLeftPosition();
      double rightPos = GetRightPosition();
      // SmartDashboard::PutNumber("elevPos", leftPos);  // print to Shuffleboard
      if(leftPos - rightPos > kElevatorDeadzone) {
        left.Set(ctre::phoenix::motorcontrol::ControlMode::Position, rightPos >= 0.0 ? rightPos : 0.0);
      } else {
        left.Set(ctre::phoenix::motorcontrol::ControlMode::Position, position);
      }
      if(rightPos - leftPos > kElevatorDeadzone) {
        left.Set(ctre::phoenix::motorcontrol::ControlMode::Position, leftPos >= 0.0 ? leftPos : 0.0);
      } else {
        right.Set(ctre::phoenix::motorcontrol::ControlMode::Position, position);
      }
    }
}

void ElevatorSubsystem::Off() {
  state = kOff;
}

void ElevatorSubsystem::On() {
  state = kPowerMode;
}

void ElevatorSubsystem::SetPower(double newPower) {
  power = newPower;
}

void ElevatorSubsystem::SetState(int newState) {
  state = newState;
}

int ElevatorSubsystem::GetState() {
  return state;
}

void ElevatorSubsystem::SetTargetPosition(double newPosition) {
  position = newPosition;
  if(position < 0) position = 0;
}

double ElevatorSubsystem::GetLeftPosition() {
  return left.GetSelectedSensorPosition(0);
}

double ElevatorSubsystem::GetRightPosition() {
  return left.GetSelectedSensorPosition(0);
}

bool ElevatorSubsystem::IsAtTarget() {
  double leftPos = GetLeftPosition();
  double rightPos = GetRightPosition();
  bool leftAtTarget = leftPos > position - (kPositionDeadzone / 2) && leftPos < position + (kPositionDeadzone / 2);
  bool rightAtTarget = rightPos > position - (kPositionDeadzone / 2) && rightPos < position + (kPositionDeadzone / 2);
  return leftAtTarget && rightAtTarget;
}

void ElevatorSubsystem::SetBrakeMode(bool state) {
  ctre::phoenix::motorcontrol::NeutralMode mode;
  if(state) mode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
  else mode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
  left.SetNeutralMode(mode);
  right.SetNeutralMode(mode);
}

void ElevatorSubsystem::ConfigMotors() {
  left.Config_kP(0, kP, 100);
  right.Config_kP(0, kP, 100);
}