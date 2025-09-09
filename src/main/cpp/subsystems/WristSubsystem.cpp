// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/WristSubsystem/WristSubsystem.h"
#include "units/angle.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace WristConstants;
using namespace frc;

WristSubsystem::WristSubsystem()
  : PositionalSubsystem{std::vector<SmartMotor*>{&wristController}},
    wrist{kWristPort} {
      ConfigWrist();
      SetTargetDegrees(kWristStartAngle);

      SmartDashboard::PutNumber("SetWristTarget", 90.0);
      SmartDashboard::PutNumber("NudgeWrist", 0.0);  // print to Shuffleboard
}


units::angle::degree_t WristSubsystem::ToDegrees(units::angle::turn_t turns) {
  return units::angle::degree_t{turns.value() / kTurnsPerDegree};
}

units::angle::turn_t WristSubsystem::ToTurns(units::angle::degree_t degrees) {
  return units::angle::turn_t{degrees.value() * kTurnsPerDegree};
}

void WristSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Wrist Control
  SetNudge(ToTurns(units::angle::degree_t{SmartDashboard::GetNumber("NudgeWrist", 0.0)}));  // print to Shuffleboard

  double feedForward = fabs(sin(ToDegrees(position).value())) * kMaxFeedForward;
  SetTargetDegrees(units::angle::degree_t{SmartDashboard::GetNumber("SetWristTarget", GetAngleDegrees().value())}, feedForward);
    // feed forwards should be a changing constant that increases as the wrist moves further. It should be a static amount of power to overcome gravity.
  SmartDashboard::PutNumber("WristActual", GetAngleDegrees().value());  // print to Shuffleboard
  SmartDashboard::PutNumber("WristTr", GetPosition().value());  // print to Shuffleboard
  SmartDashboard::PutNumber("WristTarget", ToDegrees(position).value());
  SmartDashboard::PutNumber("WristTargetTr", position.value());

  RunMotors();
}

void WristSubsystem::SetTargetDegrees(units::angle::degree_t newAngle, double feedForward) {
  newAngle = newAngle + ToDegrees(nudge) - kWristStartAngle;
  if(newAngle < kWristDegreeMin) newAngle = kWristDegreeMin;
  if(newAngle > kWristDegreeMax) newAngle = kWristDegreeMax;
  SetTargetPosition(ToTurns(newAngle), feedForward);
  SmartDashboard::PutNumber("SetWristTarget", newAngle.value());
}

units::angle::degree_t WristSubsystem::GetAngleDegrees() {
  return ToDegrees(GetPosition()) + kWristStartAngle;
}

bool WristSubsystem::IsAtTarget() {
  auto target = ToDegrees(position + nudge);
  auto angle = GetAngleDegrees();
  bool atTarget = angle > target - (kWristAngleDeadzone / 2) && angle < target + (kWristAngleDeadzone / 2);
  return atTarget;
}

frc2::CommandPtr WristSubsystem::GetMoveCommand(units::angle::degree_t target) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, target]() {
        SetTargetDegrees(target);
      }, {this}),
      frc2::cmd::WaitUntil([this](){
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
