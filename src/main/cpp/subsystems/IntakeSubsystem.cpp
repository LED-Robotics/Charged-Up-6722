// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

using namespace IntakeConstants;
using namespace frc;

IntakeSubsystem::IntakeSubsystem()
    : motor{kMotorPort} {
      motor.SetInverted(true);
}

void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  if(state == kOff) {
    motor.Set(0.0);
  } else if(state == kFullMode) {
    motor.Set(kFullPower);
  } else if(state == kPowerMode) {
    motor.Set(power);
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
  if(power < kIntakeDeadzone) power = 0.0;
}

double IntakeSubsystem::GetPower() {
  return power;
}

int IntakeSubsystem::GetState() {
  return state;
}