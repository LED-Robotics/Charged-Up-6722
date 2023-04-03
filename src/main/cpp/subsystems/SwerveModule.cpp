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
}

units::meter_t SwerveModule::GetDriveEncoderDistance() const {
    return units::meter_t{driveMotor->GetSelectedSensorPosition() * DriveConstants::kDriveEncoderDistancePerPulse};
}

units::degree_t SwerveModule::GetTurnEncoderAngle() const {
    return units::degree_t{turnMotor->GetSelectedSensorPosition() * DriveConstants::kTurnEncoderDegreesPerPulse};
}

units::meters_per_second_t SwerveModule::GetDriveEncoderRate() const {
    return units::meters_per_second_t{driveMotor->GetSelectedSensorVelocity() * DriveConstants::kDriveEncoderDistancePerPulse * 10};
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
    
    turnMotor->Set(TalonFXControlMode::Position, (double)state.angle.Degrees() / DriveConstants::kTurnEncoderDegreesPerPulse);

    driveMotor->Set(TalonFXControlMode::Velocity, ((double)state.speed / 10.0) / DriveConstants::kDriveEncoderDistancePerPulse);    
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