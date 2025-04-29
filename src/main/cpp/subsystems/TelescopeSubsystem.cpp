// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TelescopeSubsystem/TelescopeSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace TelescopeConstants;
using namespace frc;

TelescopeSubsystem::TelescopeSubsystem()
  : left{kLeftMotorPort},
  right{kRightMotorPort}
  {
    SmartDashboard::PutNumber("Telescope Position", position.value());
    SmartDashboard::PutNumber("microAdjustTelescope", 0.0);  // print to Shuffleboard
    /*SmartDashboard::PutNumber("Telescope Power", 0.0);*/
    ConfigMotors();
    SetTargetPosition(position);

}

void TelescopeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  SetTargetPosition(units::length::meter_t{SmartDashboard::GetNumber("Telescope Position", position.value())});
  /*SetPower(SmartDashboard::GetNumber("Telescope Power", power));*/
  /*SmartDashboard::PutNumber("Telescope Voltage", left.GetMotorVoltage().GetValueAsDouble());*/
  SmartDashboard::PutNumber("Left Actual Telescope", GetLeftPosition().value());
  SmartDashboard::PutNumber("Right Actual Telescope", GetRightPosition().value());
  if(state == kOff) {
    left.Set(0.0);
    right.Set(0.0);
  } else if(state == TelescopeStates::kPowerMode) {
    left.Set(power);
    right.Set(power);
  } else if(state == TelescopeStates::kPositionMode) {

    microAdjust = units::length::meter_t{SmartDashboard::GetNumber("microAdjustTelescope", 0.0)};  // print to Shuffleboard
    SmartDashboard::PutNumber("leftTelescopeTr", left.GetPosition().GetValue().value());
    SmartDashboard::PutNumber("rightTelescopeTr", right.GetPosition().GetValue().value());
    SmartDashboard::PutNumber("telescopePosition", ((GetLeftPosition().value()) + (GetRightPosition().value())) / 2);  // print to Shuffleboard
    
    SmartDashboard::PutNumber("Position Target", position.value());
    units::angle::turn_t posTarget{(position + microAdjust - kStartPosition).value() * kTurnsPerMeter};
    SmartDashboard::PutNumber("telescopeTargetTr", posTarget.value());
    
    left.SetControl(positionController
      .WithPosition(units::angle::turn_t{posTarget})
      .WithEnableFOC(true));
    right.SetControl(positionController
      .WithPosition(units::angle::turn_t{posTarget})
      .WithEnableFOC(true));

    // Test Motion Magic
    // left.SetControl(position
    //   .WithPosition(units::angle::turn_t{posTarget})
    //   .WithEnableFOC(true));
    // right.SetControl(position
    //   .WithPosition(units::angle::turn_t{posTarget})
    //   .WithEnableFOC(true));
  }
}

void TelescopeSubsystem::Off() {
  state = TelescopeStates::kOff;
}

void TelescopeSubsystem::On() {
  state = TelescopeStates::kPowerMode;
}

void TelescopeSubsystem::SetPower(double newPower) {
  power = newPower;
}

void TelescopeSubsystem::SetState(int newState) {
  state = newState;
}

int TelescopeSubsystem::GetState() {
  return state;
}

units::length::meter_t TelescopeSubsystem::GetLeftPosition() {
  auto base = units::length::meter_t{left.GetPosition().GetValueAsDouble() / kTurnsPerMeter};
  return base + kStartPosition;
}

units::length::meter_t TelescopeSubsystem::GetRightPosition() {
  auto base = units::length::meter_t{right.GetPosition().GetValueAsDouble() / kTurnsPerMeter};
  return base + kStartPosition;
}

units::length::meter_t TelescopeSubsystem::GetPosition() {
  auto left = GetLeftPosition();
  auto right = GetRightPosition();
  return (left + right) / 2;
}

void TelescopeSubsystem::SetTargetPosition(units::length::meter_t newPosition) {
  position = newPosition;
  if(position < kTelescopeMeterMin) position = kTelescopeMeterMin;
  if(position > kTelescopeMeterMax) position = kTelescopeMeterMax;
  SmartDashboard::PutNumber("Telescope Position", position.value());
}

bool TelescopeSubsystem::IsAtTarget() {
  auto target = position + microAdjust;
  auto leftPos = GetLeftPosition();
  auto rightPos = GetRightPosition();
  
  bool leftAtTarget = leftPos > target - (kPositionDeadzone / 2) && leftPos < target + (kPositionDeadzone / 2);
  bool rightAtTarget = rightPos > target - (kPositionDeadzone / 2) && rightPos < target + (kPositionDeadzone / 2);
  return leftAtTarget && rightAtTarget;
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
}

frc2::CommandPtr TelescopeSubsystem::GetMoveCommand(units::length::meter_t target) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, target]() {
        SetTargetPosition(target);
      }, {this}),
      frc2::cmd::WaitUntil([this, target](){
        return IsAtTarget();
      }));
}
