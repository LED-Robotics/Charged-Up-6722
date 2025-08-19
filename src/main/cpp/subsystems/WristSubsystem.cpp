// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/WristSubsystem/WristSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace WristConstants;
using namespace frc;

WristSubsystem::WristSubsystem()
  : wrist{kWristPort} {
      /*wrist.SetPosition(0.0_tr);*/
      SmartDashboard::PutNumber("SetWristAngle", 90.0);
      SmartDashboard::PutNumber("microAdjustWrist", 0.0);  // print to Shuffleboard
      ConfigWrist();

      SetTargetAngle(angle);

}

void WristSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Wrist Control
  SetTargetAngle(units::angle::degree_t{SmartDashboard::GetNumber("SetWristAngle", GetAngle().value())});
  SmartDashboard::PutNumber("Wrist Actual", GetAngle().value());
  if(state == WristStates::kWristOff) {
    wrist.Set(0.0);
  } else if(state == WristStates::kWristPowerMode) {
    wrist.Set(power);
  } else if(state == WristStates::kWristAngleMode) {
    // feed forwards should be a changing constant that increases as the wrist moves further. It should be a static amount of power to overcome gravity.

    microAdjust = units::angle::degree_t{SmartDashboard::GetNumber("microAdjustWrist", 0.0)};  // print to Shuffleboard
    SmartDashboard::PutNumber("wristTr", wrist.GetPosition().GetValue().value());  // print to Shuffleboard
    SmartDashboard::PutNumber("wristAngle", GetAngle().value());  // print to Shuffleboard
    double feedForward = fabs(sin(angle.value())) * kMaxFeedForward;
    SmartDashboard::PutNumber("wristTarget", angle.value());
    units::angle::turn_t posTarget{(angle + microAdjust - kWristStartAngle).value() * kTurnsPerDegree};
    SmartDashboard::PutNumber("wristTrTarget", posTarget.value());
    wrist.SetControl(wristPosition
      .WithPosition(units::angle::turn_t{posTarget})
      .WithEnableFOC(true));
      /*.WithFeedForward(units::volt_t{feedForward}));*/
  }
}

void WristSubsystem::WristOn() {
  state = WristStates::kWristAngleMode;
}

void WristSubsystem::WristOff() {
  state = WristStates::kWristOff;
}

void WristSubsystem::SetWristPower(double newPower) {
  power = newPower;
}

double WristSubsystem::GetWristPower() {
  return power;
}

void WristSubsystem::SetTargetAngle(units::angle::degree_t newAngle) {
  angle = newAngle;
  if(angle < kWristDegreeMin) angle = kWristDegreeMin;
  if(angle > kWristDegreeMax) angle = kWristDegreeMax;
  SmartDashboard::PutNumber("Wrist Angle", angle.value());
}

units::angle::degree_t WristSubsystem::GetAngle() {
  return units::angle::degree_t{(GetWristPosition() / kTurnsPerDegree)} + kWristStartAngle;
}

double WristSubsystem::GetWristPosition() {
  return wrist.GetPosition().GetValueAsDouble();
}

bool WristSubsystem::IsAtTarget() {
  auto target = angle + microAdjust;
  auto angle = GetAngle();
  bool atTarget = angle > target - (kWristAngleDeadzone / 2) && angle < target + (kWristAngleDeadzone / 2);
  return atTarget;
}

void WristSubsystem::SetWristState(int newState) {
  state = newState;
}

int WristSubsystem::GetWristState() {
  return state;
}

frc2::CommandPtr WristSubsystem::GetMoveCommand(units::angle::degree_t target) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, target]() {
        SetTargetAngle(target);
      }, {this}),
      frc2::cmd::WaitUntil([this, target](){
        return IsAtTarget();
      }));
  /*return frc2::cmd::RunOnce([this, target]() {*/
  /*      SetTargetAngle(target);*/
  /*    }, {this});*/
}
void WristSubsystem::SetWristBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);
  wrist.GetConfigurator().Apply(updated, 50_ms);
}

void WristSubsystem::ConfigWrist() {
  configs::TalonFXConfiguration wristConfig{};

  wristConfig.Slot0.kP = kPWrist;
  wristConfig.Slot0.kD = kDWrist;
  wristConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
  // wristConfig.Slot0.kS = 0.28;
  // wristConfig.Slot0.kV = 8.5;
  // wristConfig.Slot0.kA = 3.0;
  // wristConfig.Slot0.kP = 8.0;

  // wristConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
  // wristConfig.MotionMagic.MotionMagicAcceleration = 2.0;
  // wristConfig.MotionMagic.MotionMagicJerk = 200.0;
  
  // wristConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
  wristConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
  wristConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
  // wristConfig.Feedback.SensorToMechanismRatio = 1.0;
  wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  wristConfig.CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
  wristConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
  wristConfig.Audio.AllowMusicDurDisable = true;

  
  wrist.GetConfigurator().Apply(wristConfig);

}
