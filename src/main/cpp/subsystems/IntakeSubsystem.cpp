// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

using namespace IntakeConstants;
using namespace frc;

IntakeSubsystem::IntakeSubsystem()
    : intakeMotor{kIntakePort, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    wristMotor{kWristPort} {
      intakeMotor.SetInverted(true);
}

void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here

  // Intake power control
  if(state == kOff) {
    intakeMotor.Set(0.0);
  } else if(state == kFullMode) {
    if(intakeMotor.GetOutputCurrent() < kCurrentLimit) intakeMotor.Set(kFullPower);
  } else if(state == kPowerMode) {
    if(intakeMotor.GetOutputCurrent() < kCurrentLimit) intakeMotor.Set(power);
  }

  // Wrist position control
  // feed forward should be a changing constant that increases as the wrist moves further. It should be a static amount of power to overcome gravity.
  double feedForward = 0.0; // GetCurrentPosition() / 5000   <-- tune this number after verifying the motion magic works in any capacity
  wristMotor.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, position, ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, feedForward);
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

void IntakeSubsystem::SetPosition(double newPosition) {
  position = newPosition;
}

double IntakeSubsystem::GetTargetPosition() {
  return position;
}

double IntakeSubsystem::GetCurrentPosition() {
  return wristMotor.SetSelectedSensorPosition(0);
}

void IntakeSubsystem::ResetWristEncoder() {
  wristMotor.SetSelectedSensorPosition(0);
}