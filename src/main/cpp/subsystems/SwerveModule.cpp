// Copyright (c) FIRST and other WPILib contributors.
// Open Soruce Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in root directory of this project.

#include "subsystems/SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

SwerveModule::SwerveModule(WPI_TalonFX *drivingMotor,
                            WPI_TalonFX *turningMotor) {
    driveMotor = drivingMotor;
    turnMotor = turningMotor;
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    // m_driveEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius / kEncoder Resolution);

    // Set the distance (in thie case, angle) per pulse for the turning encoder.
    // This is the angle through an entire rotation (2 * std::numbers::pi)
    // drivided by the encoder resolution.
    
    // m_turningEncoder.SetDistancePerPulse(2 * std::number::pi / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.EnableContinuousInput(
        -units::degree_t{180}, units::degree_t{180});
}

units::meter_t SwerveModule::GetDriveEncoderDistance() const {
    return units::meter_t{driveMotor->GetSelectedSensorPosition() * DriveConstants::kDriveEncoderDistancePerPulse};
}

units::degree_t SwerveModule::GetTurnEncoderAngle() const {
    return units::degree_t{turnMotor->GetSelectedSensorPosition() * DriveConstants::kTurnEncoderDegreesPerPulse};
}

units::meters_per_second_t SwerveModule::GetDriveEncoderRate() const {
    return units::meters_per_second_t{driveMotor->GetSelectedSensorVelocity() * DriveConstants::kDriveEncoderDistancePerPulse};
}

frc::SwerveModuleState SwerveModule::GetState() const {
    return {GetDriveEncoderRate(),
            GetTurnEncoderAngle()};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
    return {GetDriveEncoderDistance(),
            GetTurnEncoderAngle()};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
    // Optimize the reference state to avoid spinning further than 90 degrees*

    const auto state = frc::SwerveModuleState::Optimize(referenceState, GetTurnEncoderAngle());
    // last = &referenceState;
    // Calculate the drive output from the drive PID controller.
    // const auto driveOutput = drivePIDController.Calculate(
    //     driveMotor->GetSelectedSensorVelocity(), state.speed.value());
    
    // const auto driveFeedforwardState = driveFeedforward.Calculate(state.speed);

    // Calculate the turning motor output from the turning PID controller.
    // const auto turnOutput = turningPIDController.Calculate(
    //     GetTurnEncoderAngle(), state.angle.Degrees());

    // const auto turnFeedforwardState = turnFeedforward.Calculate(
    //     turningPIDController.GetSetpoint().velocity);

    // Set the motor outputs.
    // driveMotor->SetVoltage(units::volt_t{driveOutput} + driveFeedforwardState);
    // turnMotor->SetVoltage(units::volt_t{turnOutput} + turnFeedforwardState);
    driveMotor->Set(TalonFXControlMode::Velocity, ((double)state.speed / 10.0) / DriveConstants::kDriveEncoderDistancePerPulse);

    // driveMotor->Set(TalonFXControlMode::Velocity, ((double)state.speed / 10.0) / DriveConstants::kDriveEncoderDistancePerPulse,
    // ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, 0.3);
    
    turnMotor->Set(TalonFXControlMode::Position, (double)state.angle.Degrees() / DriveConstants::kTurnEncoderDegreesPerPulse);
}

void SwerveModule::SetDrivePower(double power) {
    driveMotor->Set(power);
}

void SwerveModule::SetTurnPower(double power) {
    turnMotor->Set(power);
}

void SwerveModule::ResetEncoders() {
    driveMotor->SetSelectedSensorPosition(0);
    turnMotor->SetSelectedSensorPosition(0);
}