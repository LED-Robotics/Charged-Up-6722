// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TelescopeSubsystem/TelescopeSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace TelescopeConstants;
using namespace frc;

TelescopeSubsystem::TelescopeSubsystem()
  : PositionalSubsystem{std::vector<SmartMotor*>{&leftController, &rightController}},
  left{kLeftMotorPort, "canCan"},
  right{kRightMotorPort, "canCan"}
  {
    ConfigMotors();
    SetTargetMeters(kStartPosition);
    SetState(kPositionMode);

    SmartDashboard::PutNumber("SetTelescopeTarget", position.value());
    SmartDashboard::PutNumber("NudgeTelescope", 0.0);  // print to Shuffleboard
}


units::length::meter_t TelescopeSubsystem::ToMeters(units::angle::turn_t turns) {
  return units::length::meter_t{turns.value() / kTurnsPerMeter};
}

units::angle::turn_t TelescopeSubsystem::ToTurns(units::length::meter_t meters) {
  return units::angle::turn_t{meters.value() * kTurnsPerMeter};
}


void TelescopeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  SetNudge(ToTurns(units::length::meter_t{SmartDashboard::GetNumber("NudgeTelescope", 0.0)}));  // print to Shuffleboard
  SetTargetMeters(units::length::meter_t{SmartDashboard::GetNumber("SetTelescopeTarget", ToMeters(position).value())});

  SmartDashboard::PutNumber("TelescopeActual", ToMeters(GetPosition()).value());  // print to Shuffleboard
  SmartDashboard::PutNumber("leftTelescopeTr", GetLeftPosition().value());
  SmartDashboard::PutNumber("rightTelescopeTr", GetRightPosition().value());
  SmartDashboard::PutNumber("TelescopeTarget", ToMeters(position).value());
  SmartDashboard::PutNumber("TelescopeTargetTr", position.value());

  
  RunMotors();
}

units::length::meter_t TelescopeSubsystem::GetPositionMeters() {
  return ToMeters(GetPosition());
}

void TelescopeSubsystem::SetTargetMeters(units::length::meter_t newPosition) {
  if(newPosition < kTelescopeMeterMin) newPosition = kTelescopeMeterMin;
  if(newPosition > kTelescopeMeterMax) newPosition = kTelescopeMeterMax;
  SetTargetPosition(ToTurns(newPosition));
  SmartDashboard::PutNumber("SetTelescopeTarget", newPosition.value());
}

bool TelescopeSubsystem::IsAtTarget() {
  auto target = ToMeters(position + nudge);
  auto pos = GetPositionMeters();
  
  bool atTarget = pos > target - (kPositionDeadzone / 2) && pos < target + (kPositionDeadzone / 2);
  return atTarget;
}

void TelescopeSubsystem::SetBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);

  left.GetConfigurator().Apply(updated, 50_ms);
  right.GetConfigurator().Apply(updated, 50_ms);
}

void TelescopeSubsystem::ConfigMotors() {
  configs::TalonFXConfiguration telescopeConfig{};
  
  telescopeConfig.Slot0.kP = kP;
  telescopeConfig.Slot0.kD = kD;
  telescopeConfig.Slot0.kG = kG;
  // telescopeConfig.Slot0.kS = 0.28;
  // telescopeConfig.Slot0.kV = 8.5;
  // telescopeConfig.Slot0.kA = 3.0;
  // telescopeConfig.Slot0.kP = 8.0;

  // telescopeConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
  // telescopeConfig.MotionMagic.MotionMagicAcceleration = 2.0;
  // telescopeConfig.MotionMagic.MotionMagicJerk = 200.0;
  
  telescopeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  telescopeConfig.CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
  telescopeConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
  telescopeConfig.Feedback.RotorToSensorRatio = kRotorToGearbox;
  telescopeConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
  telescopeConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
  telescopeConfig.Feedback.SensorToMechanismRatio = 16.0;
  telescopeConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
  telescopeConfig.Audio.AllowMusicDurDisable = true;
  
  telescopeConfig.MotorOutput.Inverted = false;
  // telescopeConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
  
  left.GetConfigurator().Apply(telescopeConfig);
  // telescopeConfig.DifferentialSensors.DifferentialSensorSource = signals::DifferentialSensorSourceValue::RemoteTalonFX_Diff;
  // telescopeConfig.DifferentialSensors.DifferentialTalonFXSensorID = kLeftMotorPort;
  telescopeConfig.MotorOutput.Inverted = true;

  right.GetConfigurator().Apply(telescopeConfig);

  // configs::CANcoderConfiguration encoderConfig{};
  // encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
  // encoderConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
  // encoderConfig.MagnetSensor.MagnetOffset = kEncoderOffset;
  // encoder.GetConfigurator().Apply(encoderConfig);
}

frc2::CommandPtr TelescopeSubsystem::GetMoveCommand(units::length::meter_t target) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, target]() {
        SetTargetMeters(target);
      }, {this}),
      frc2::cmd::WaitUntil([this](){
        return IsAtTarget();
      }));
}

// For debug
units::length::meter_t TelescopeSubsystem::GetLeftPosition() {
  auto base = units::length::meter_t{left.GetPosition().GetValueAsDouble() / kTurnsPerMeter};
  return base + kStartPosition;
}

units::length::meter_t TelescopeSubsystem::GetRightPosition() {
  auto base = units::length::meter_t{right.GetPosition().GetValueAsDouble() / kTurnsPerMeter};
  return base + kStartPosition;
}

