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
      wristMotor.SetSelectedSensorPosition(0);
}

void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  std::cout << "Wrist Current Position: " << GetCurrentPosition() << '\n';
  // std::cout << "Intake Output Current: " << intakeMotor.GetOutputCurrent() << '\n';
  // Intake power control
  if(state == kOff) {
    intakeMotor.Set(0.0);
  } else if(state == kPowerMode) {
    // if(intakeMotor.GetOutputCurrent() < kCurrentLimit && power > 0.0) intakeMotor.Set(power);
    intakeMotor.Set(power);
    // else intakeMotor.Set(0.0);
  }
  // std::cout << "Wrist Position Target: " << position << '\n';
  // Wrist position control
  // feed forward should be a changing constant that increases as the wrist moves further. It should be a static amount of power to overcome gravity.
  double feedForward = 0.0; // GetCurrentPosition() / 5000   <-- tune this number after verifying the motion magic works in any capacity
  // double feedForward = sin((GetCurrentPosition() / kCountsPerDegree) * (M_PI/180)) * kMaxFeedForward;
  wristMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, position, ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, feedForward);
  // wristMotor.Set(wristPower);
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
  // if(power < kIntakeDeadzone) power = 0.0;
}

void IntakeSubsystem::SetWristPower(double newPower) {
  wristPower = newPower;
  // if(power < kIntakeDeadzone) power = 0.0;
}

double IntakeSubsystem::GetPower() {
  return power;
}

int IntakeSubsystem::GetState() {
  return state;
}

void IntakeSubsystem::SetState(int newState) {
  state = newState;
}

void IntakeSubsystem::SetPosition(double newPosition) {
  position = newPosition;
}

double IntakeSubsystem::GetTargetPosition() {
  return position;
}

double IntakeSubsystem::GetCurrentPosition() {
  return wristMotor.GetSelectedSensorPosition();
}

void IntakeSubsystem::ResetWristEncoder() {
  wristMotor.SetSelectedSensorPosition(0);
}