// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretSubsystem.h"
#include <iostream>

using namespace TurretConstants;
using namespace frc;

TurretSubsystem::TurretSubsystem()
    : motor{kMotorPort},
    encoder{kEncoderAPort, kEncoderBPort, kEncoderReversed},
    pidController{kP, kI, 0.0} {
  ResetEncoder();
}

void TurretSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  if(state == kOff) {
    motor.Set(0.0);
  } else if(state == kPowerMode) {
    if(power > 0.0 && GetPosition() < kMaxRight) {
      motor.Set(power);
    } else if(power < 0.0 && GetPosition() > kMaxLeft) {
      motor.Set(power);
    }
  } else if(state == kPositionMode) {
    double value = pidController.Calculate(GetPosition(), target);
    if(value > 0.0 && GetPosition() < kMaxRight) {
      motor.Set(value);
    } else if(value < 0.0 && GetPosition() > kMaxLeft) {
      motor.Set(value);
    }
  }
}

void TurretSubsystem::ResetEncoder() {
  encoder.Reset();
}

void TurretSubsystem::SetPower(double newPower) {
  power = newPower;
}

double TurretSubsystem::GetPower() {
  return power;
}

void TurretSubsystem::SetTarget(double newTarget) {
  target = newTarget;
}

int TurretSubsystem::GetTarget() {
  return target;
}

int TurretSubsystem::GetPosition() {
  return encoder.Get();
}

void TurretSubsystem::Off() {
  state = kOff;
}

void TurretSubsystem::SetState(int newState) {
  state = newState;
}

int TurretSubsystem::GetState() {
  return state;
}