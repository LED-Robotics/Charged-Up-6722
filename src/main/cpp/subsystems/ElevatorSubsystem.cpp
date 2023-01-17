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
    : bottom{kMotor1Port},
    top{kMotor2Port} {
}

void ElevatorSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  if(state == kOff) {
    bottom.Set(0.0);
    top.Set(0.0);
  } else if(state == kOn) {
    bottom.Set(power);
    top.Set(power);
  }
}

void ElevatorSubsystem::Off() {
  state = kOff;
}

void ElevatorSubsystem::On() {
  state = kOn;
}

void ElevatorSubsystem::SetPower(double newPower) {
  power = newPower;
}

int ElevatorSubsystem::GetState() {
  return state;
}