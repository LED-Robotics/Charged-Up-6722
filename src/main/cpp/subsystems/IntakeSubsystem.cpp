// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace IntakeConstants;
using namespace frc;

IntakeSubsystem::IntakeSubsystem(ArmSubsystem *reference)
    : intakeMotor{kIntakePort},
    wristMotor{kWristPort} {
      arm = reference;
      intakeMotor.SetInverted(true);
      wristMotor.SetSelectedSensorPosition(0);
}

void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Intake power control
  if(state == kOff) {
    intakeMotor.Set(0.0);
  } else if(state == kPowerMode) {
    // if(intakeMotor.GetOutputCurrent() < kCurrentLimit && power > 0.0) intakeMotor.Set(power);
    intakeMotor.Set(power);
    // else intakeMotor.Set(0.0);
  }

  // Wrist position control
  double armAngle = arm->GetAngle();
  // std::cout << "Arm Angle: " << armAngle << '\n';
  // feed forward should be a changing constant that increases as the wrist moves further. It should be a static amount of power to overcome gravity.
  double wristAngle = (kStartAngle - armAngle + (GetCurrentPosition() / kCountsPerDegree)) * (M_PI/180);
  // double feedForward = 0.0;
  double feedForward = sin(wristAngle) * kMaxFeedForward;
  SmartDashboard::PutNumber("wristAngle", (kStartAngle - armAngle + (GetCurrentPosition() / kCountsPerDegree)));  // print to Shuffleboard
  wristMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, position, ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, feedForward);
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

bool IntakeSubsystem::IsAtTarget() {
  double pos = GetCurrentPosition();
  bool atTarget = pos > position - (kPositionDeadzone / 2) && pos < position + (kPositionDeadzone / 2);
  return atTarget;
}

void IntakeSubsystem::SetBrakeMode(bool state) {
  ctre::phoenix::motorcontrol::NeutralMode mode;
  if(state) mode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
  else mode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
  intakeMotor.SetNeutralMode(mode);
  wristMotor.SetNeutralMode(mode);
}