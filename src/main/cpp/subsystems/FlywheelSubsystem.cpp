// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FlywheelSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

using namespace FlywheelConstants;
using namespace frc;

FlywheelSubsystem::FlywheelSubsystem()
    : motor1{kMotor1Port} {
  motor1.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0);
  ResetEncoders();
}

void FlywheelSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  if(state == kOff) {
    motor1.Set(0.0);
  } else if(state == kPowerMode) {
    std::cout << "Flywheel RPM: " << GetRPM() << '\n';
    motor1.Set(power);
  } else if(state == kRpmMode) {
    std::cout << "Flywheel RPM: " << GetRPM() << '\n';
    motor1.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity, rpm);
  }
}

void FlywheelSubsystem::ResetEncoders() {
  motor1.SetSelectedSensorPosition(0);
}

void FlywheelSubsystem::Off() {
  state = kOff;
}

void FlywheelSubsystem::SetMotorRPM(double newRPM) {
  rpm = newRPM;
}

void FlywheelSubsystem::SetMotorPower(double newPower) {
  power = newPower;
}

double FlywheelSubsystem::GetMotorPower() {
  return power;
}

void FlywheelSubsystem::SetFlywheelState(int newState) {
  state = newState;
}

int FlywheelSubsystem::GetFlywheelState() {
  return state;
}

double FlywheelSubsystem::GetRPM() {
  return motor1.GetSelectedSensorVelocity() * 200 / 2048;
}