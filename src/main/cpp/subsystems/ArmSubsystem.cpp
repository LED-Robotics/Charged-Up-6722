// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem/ArmSubsystem.h"
#include "led_libraries/PositionalSubsystem.h"
#include "units/angle.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ArmConstants;
using namespace frc;

ArmSubsystem::ArmSubsystem()
  : PositionalSubsystem{std::vector<SmartMotor*>{&left, &right}},
    left{kLeftArmPort},
    right{kRightArmPort} {
      ConfigArm();
      SetTargetDegrees(kArmStartAngle);
      SetState(kProfileMode);

      SmartDashboard::PutNumber("SetArmTarget", ToDegrees(position).value());
      SmartDashboard::PutNumber("NudgeArm", 0.0);  // print to Shuffleboard
}


units::angle::degree_t ArmSubsystem::ToDegrees(units::angle::turn_t turns) {
  return units::angle::degree_t{turns.value() / kTurnsPerDegree};
}

units::angle::turn_t ArmSubsystem::ToTurns(units::angle::degree_t degrees) {
  return units::angle::turn_t{degrees.value() * kTurnsPerDegree};
}

void ArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Arm Control
  SetNudge(ToTurns(units::angle::degree_t{SmartDashboard::GetNumber("NudgeArm", 0.0)}));  // print to Shuffleboard

  // double feedForward = fabs(sin(ToDegrees(position).value())) * kMaxFeedForward;
  double feedForward = 0.0;
  SetTargetDegrees(units::angle::degree_t{SmartDashboard::GetNumber("SetArmTarget", ToDegrees(position).value())}, feedForward);
    // feed forwards should be a changing constant that increases as the arm moves further. It should be a static amount of power to overcome gravity.
  SmartDashboard::PutNumber("ArmActual", GetAngleDegrees().value());  // print to Shuffleboard
  SmartDashboard::PutNumber("ArmTr", GetPosition().value());  // print to Shuffleboard
  SmartDashboard::PutNumber("ArmTarget", ToDegrees(position).value());
  SmartDashboard::PutNumber("ArmTargetTr", position.value());

  RunMotors();
}

void ArmSubsystem::SetTargetDegrees(units::angle::degree_t newAngle, double feedForward) {
  if(newAngle < kArmDegreeMin) newAngle = kArmDegreeMin;
  if(newAngle > kArmDegreeMax) newAngle = kArmDegreeMax;
  SetTargetPosition(ToTurns(newAngle), feedForward);
  SmartDashboard::PutNumber("SetArmTarget", newAngle.value());
}

units::angle::degree_t ArmSubsystem::GetAngleDegrees() {
  return ToDegrees(GetPosition()) + kArmStartAngle;
}

bool ArmSubsystem::IsAtTarget() {
  auto target = ToDegrees(position + nudge);
  auto angle = GetAngleDegrees();
  bool atTarget = angle > target - (kArmAngleDeadzone / 2) && angle < target + (kArmAngleDeadzone / 2);
  return atTarget;
}

frc2::CommandPtr ArmSubsystem::GetMoveCommand(units::angle::degree_t target) {
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
void ArmSubsystem::SetArmBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);
  left.motor.GetConfigurator().Apply(updated, 50_ms);
  right.motor.GetConfigurator().Apply(updated, 50_ms);
}

void ArmSubsystem::ConfigArm() {
  configs::TalonFXConfiguration armConfig{};

  armConfig.Slot0.kP = kPArm;
  armConfig.Slot0.kD = kDArm;
  armConfig.Slot0.kV = kVArm;
  armConfig.Slot0.kA = kAArm;
  armConfig.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
  // armConfig.Slot0.kS = 0.28;
  // armConfig.Slot0.kP = 8.0;

  armConfig.MotionMagic.MotionMagicCruiseVelocity = kMotionMagicCruiseVelocity;
  armConfig.MotionMagic.MotionMagicAcceleration = kMotionMagicAcceleration;

  // armConfig.Feedback.FeedbackSensorSource =
  // signals::FeedbackSensorSourceValue::RotorSensor;
  // armConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
  armConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
  armConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
  armConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
  // armConfig.Feedback.SensorToMechanismRatio = 1.0;
  armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  armConfig.CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
  armConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
  armConfig.Audio.AllowMusicDurDisable = true;

  // armConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;

  left.motor.GetConfigurator().Apply(armConfig);
  armConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
  right.motor.GetConfigurator().Apply(armConfig);
}
