// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LiftSubsystem.h"

#include <iostream>

using namespace LiftConstants;
using namespace frc;

LiftSubsystem::LiftSubsystem()
    : motor{kMotorPort} {
}

void LiftSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  if(state == kOff) {
    motor.Set(0.0);
  } else if(state == kForward) {
    motor.Set(kDefaultPower);
  } else if(state == kReverse) {
    motor.Set(-kDefaultPower);
  } else if(state == kPowerMode) {
    motor.Set(power);
  }
}

void LiftSubsystem::Off() {
  state = kOff;
}

void LiftSubsystem::SetState(int newState) {
  state = newState;
}

int LiftSubsystem::GetState() {
  return state;
}

void LiftSubsystem::SetPower(double newPower) {
  power = newPower;
}