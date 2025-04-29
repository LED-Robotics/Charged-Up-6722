// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem/IntakeSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace IntakeConstants;
using namespace frc;

IntakeSubsystem::IntakeSubsystem() 
    : intake{kIntakePort} {

    }


void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Coral power control

  if(state == IntakeStates::kOff) {
    intake.Set(0.0);
  } else if(state == IntakeStates::kPowerMode) {
    // power limiting 
    // if(intakeMotor.GetOutputCurrent() < kCurrentLimit && power > 0.0) intakeMotor.Set(power);
    intake.Set(power);

    // else intakeMotor.Set(0.0);
  } 
}

void IntakeSubsystem::Off() {
  state = IntakeStates::kOff;
}

void IntakeSubsystem::UsePowerMode() {
  state = IntakeStates::kPowerMode;
}

void IntakeSubsystem::SetPower(double newPower) {
  power = newPower;
  // if(power < kCoralDeadzone) power = 0.0;
}

double IntakeSubsystem::GetPower() {
  return power;
}

int IntakeSubsystem::GetState() {
  return state;
}

void IntakeSubsystem::SetState(IntakeStates newState) {
  state = newState;
}

void IntakeSubsystem::SetBrakeMode(bool state) {
  /*signals::NeutralModeValue mode;*/
  /*if(state) mode = signals::NeutralModeValue::Brake;*/
  /*else mode = signals::NeutralModeValue::Coast;*/
  /*configs::MotorOutputConfigs updated;*/
  /*updated.WithNeutralMode(mode);*/
  /**/
  /*intakeMotor.GetConfigurator().Apply(updated, 50_ms);*/
}

void IntakeSubsystem::ConfigMotors() {
  configs::TalonFXConfiguration intakeConfig{};

  // coralConfig.Slot0.kP = kP;
  intakeConfig.MotorOutput.Inverted = true;
  // coralConfig.Slot0.kS = 0.28;
  // coralConfig.Slot0.kV = 8.5;
  // coralConfig.Slot0.kA = 3.0;
  // coralConfig.Slot0.kP = 8.0;

  // coralConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
  // coralConfig.MotionMagic.MotionMagicAcceleration = 2.0;
  // coralConfig.MotionMagic.MotionMagicJerk = 200.0;
  
  // coralConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
  // coralConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
  // coralConfig.Feedback.RotorToSensorRatio = kRotorToGearbox;
  // coralConfig.Feedback.SensorToMechanismRatio = kRotorToGearbox * kGearboxToMechanism;
  // coralConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
  // coralConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
  // coralConfig.Feedback.SensorToMechanismRatio = 1.0;
  // coralConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
  // coralConfig.Audio.AllowMusicDurDisable = true;

  // coralConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
  
  intake.GetConfigurator().Apply(intakeConfig);
  intakeConfig.MotorOutput.Inverted = false;

}


