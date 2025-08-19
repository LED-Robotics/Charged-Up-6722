// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem/ArmSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

using namespace ArmConstants;
using namespace frc;

ArmSubsystem::ArmSubsystem() : leftArm{kLeftArmPort}, rightArm{kRightArmPort} {
  /*arm.SetPosition(0.0_tr);*/
  SmartDashboard::PutNumber("SetArmAngle", angle.value());
  SmartDashboard::PutNumber("microAdjustArm", 0.0); // print to Shuffleboard
  ConfigArm();

  SetTargetAngle(angle);
}

void ArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Arm Control
  SetTargetAngle(units::angle::degree_t{
      SmartDashboard::GetNumber("SetArmAngle", GetAngle().value())});
  SmartDashboard::PutNumber("Arm Actual", GetAngle().value());
  if (state == ArmStates::kArmOff) {
    leftArm.Set(0.0);
    rightArm.Set(0.0);
  } else if (state == ArmStates::kArmPowerMode) {
    leftArm.Set(power);
    rightArm.Set(power);
  } else if (state == ArmStates::kArmAngleMode) {
    // feed forwards should be a changing constant that increases as the arm
    // moves further. It should be a static amount of power to overcome gravity.

    microAdjust = units::angle::degree_t{SmartDashboard::GetNumber(
        "microAdjustArm", 0.0)}; // print to Shuffleboard
    SmartDashboard::PutNumber(
        "armTr",
        GetArmPosition()); // print to Shuffleboard
    SmartDashboard::PutNumber("armAngle",
                              GetAngle().value()); // print to Shuffleboard
    double feedForward = fabs(sin(angle.value())) * kMaxFeedForward;
    SmartDashboard::PutNumber("armTarget", angle.value());
    units::angle::turn_t posTarget{
        (angle + microAdjust - kArmStartAngle).value() * kTurnsPerDegree};
    SmartDashboard::PutNumber("armTrTarget", posTarget.value());
    leftArm.SetControl(armPosition.WithPosition(units::angle::turn_t{posTarget})
                           .WithEnableFOC(true));
    rightArm.SetControl(
        armPosition.WithPosition(units::angle::turn_t{posTarget})
            .WithEnableFOC(true));
    /*.WithFeedForward(units::volt_t{feedForward}));*/
  }
}

void ArmSubsystem::ArmOn() { state = ArmStates::kArmAngleMode; }

void ArmSubsystem::ArmOff() { state = ArmStates::kArmOff; }

void ArmSubsystem::SetArmPower(double newPower) { power = newPower; }

double ArmSubsystem::GetArmPower() { return power; }

void ArmSubsystem::SetTargetAngle(units::angle::degree_t newAngle) {
  angle = newAngle;
  if (angle < kArmDegreeMin)
    angle = kArmDegreeMin;
  if (angle > kArmDegreeMax)
    angle = kArmDegreeMax;
  SmartDashboard::PutNumber("Arm Angle", angle.value());
}

units::angle::degree_t ArmSubsystem::GetAngle() {
  return units::angle::degree_t{(GetArmPosition() / kTurnsPerDegree)} +
         kArmStartAngle;
}

double ArmSubsystem::GetArmPosition() {
  return (leftArm.GetPosition().GetValueAsDouble() + rightArm.GetPosition().GetValueAsDouble()) / 2.0;
}

bool ArmSubsystem::IsAtTarget() {
  auto target = angle + microAdjust;
  auto angle = GetAngle();
  bool atTarget = angle > target - (kArmAngleDeadzone / 2) &&
                  angle < target + (kArmAngleDeadzone / 2);
  return atTarget;
}

void ArmSubsystem::SetArmState(int newState) { state = newState; }

int ArmSubsystem::GetArmState() { return state; }

frc2::CommandPtr ArmSubsystem::GetMoveCommand(units::angle::degree_t target) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, target]() { SetTargetAngle(target); }, {this}),
      frc2::cmd::WaitUntil([this, target]() { return IsAtTarget(); }));
  /*return frc2::cmd::RunOnce([this, target]() {*/
  /*      SetTargetAngle(target);*/
  /*    }, {this});*/
}
void ArmSubsystem::SetArmBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if (state)
    mode = signals::NeutralModeValue::Brake;
  else
    mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);
  leftArm.GetConfigurator().Apply(updated, 50_ms);
  rightArm.GetConfigurator().Apply(updated, 50_ms);
}

void ArmSubsystem::ConfigArm() {
  configs::TalonFXConfiguration armConfig{};

  armConfig.Slot0.kP = kPArm;
  armConfig.Slot0.kD = kDArm;
  armConfig.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
  // armConfig.Slot0.kS = 0.28;
  // armConfig.Slot0.kV = 8.5;
  // armConfig.Slot0.kA = 3.0;
  // armConfig.Slot0.kP = 8.0;

  // armConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
  // armConfig.MotionMagic.MotionMagicAcceleration = 2.0;
  // armConfig.MotionMagic.MotionMagicJerk = 200.0;

  // armConfig.Feedback.FeedbackSensorSource =
  // signals::FeedbackSensorSourceValue::RotorSensor;
  // armConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
  armConfig.MotorOutput.PeakReverseDutyCycle = -0.4;
  armConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
  // armConfig.Feedback.SensorToMechanismRatio = 1.0;
  armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  armConfig.CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
  armConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
  armConfig.Audio.AllowMusicDurDisable = true;

  // armConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;

  leftArm.GetConfigurator().Apply(armConfig);
  armConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
  rightArm.GetConfigurator().Apply(armConfig);
}
