// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

using namespace ElevatorConstants;
using namespace frc;

ElevatorSubsystem::ElevatorSubsystem()
    : leftStopSensor{kLeftStopPort},
    rightStopSensor{kRightStopPort},
    left{kLeftMotorPort},
    right{kRightMotorPort} {
}

void ElevatorSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  if(state == kOff) {
    left.Set(0.0);
    right.Set(0.0);
  } else if(state == kPowerMode) {
    left.Set(leftStopSensor.Get() ? 0.0 : power);
    right.Set(rightStopSensor.Get() ? 0.0 : power);
  } else if(state == kPositionMode) {
    if(leftStopSensor.Get()) left.Set(0.0);
    else left.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, position);
    if(rightStopSensor.Get()) right.Set(0.0);
    else right.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, position);
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
}

double ElevatorSubsystem::GetLeftPosition() {
  return left.GetSelectedSensorPosition(0);
}

double ElevatorSubsystem::GetRightPosition() {
  return left.GetSelectedSensorPosition(0);
}