// Copyright (c) FIRST and other WPILib contributors.
// Open Soruce Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in root directory of this project.

#include "subsystems/SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

SwerveModule::SwerveModule(WPI_TalonFX *drivingMotor,
                            WPI_TalonFX *turningMotor) {
    driveMotor = drivingMotor;
    falconTurn = turningMotor;
}

SwerveModule::SwerveModule(WPI_TalonFX *drivingMotor,
                            rev::CANSparkMax *turningMotor) {
    usingFalcon = false;
    driveMotor = drivingMotor;
    neoTurn = turningMotor;
    neoController = (SparkMaxPIDController*) malloc(sizeof(SparkMaxPIDController));
    *neoController = neoTurn->GetPIDController();
}

double SwerveModule::GetFalconTurnPosition() const {
    return falconTurn->GetSelectedSensorPosition();
}

double SwerveModule::GetNeoTurnPosition() const {
    return 0.0;
}

void SwerveModule::SetFalconTurnPower(double power) {
    falconTurn->Set(power);
}

void SwerveModule::SetNeoTurnPower(double power) {
    neoTurn->Set(power);
}

units::meter_t SwerveModule::GetDriveEncoderDistance() const {
    return units::meter_t{driveMotor->GetSelectedSensorPosition() * DriveConstants::kDriveEncoderDistancePerPulse};
}

units::degree_t SwerveModule::GetTurnEncoderAngle() const {
    return units::degree_t{(usingFalcon ? GetFalconTurnPosition() : GetNeoTurnPosition()) * DriveConstants::kTurnEncoderDegreesPerPulse};
}

units::meters_per_second_t SwerveModule::GetDriveEncoderRate() const {
    return units::meters_per_second_t{driveMotor->GetSelectedSensorVelocity() * DriveConstants::kDriveEncoderDistancePerPulse * 10};
}

frc::SwerveModuleState SwerveModule::GetState() const {
    return {GetDriveEncoderRate(),
            GetTurnEncoderAngle()};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
    return {GetDriveEncoderDistance() * -1,
            GetTurnEncoderAngle()};
}

// adapted from team 364s BaseFalconSwerve example
frc::SwerveModuleState SwerveModule::Optimize(const frc::SwerveModuleState& desiredState, frc::Rotation2d currentAngle) {
    double targetAngle = PlaceInAppropriate0To360Scope((double)currentAngle.Degrees(), (double)desiredState.angle.Degrees());
    double targetSpeed = (double)desiredState.speed;
    double delta = targetAngle - (double)currentAngle.Degrees();
    if (fabs(delta) > 90.0){
        targetSpeed *= -1.0;
        targetAngle = delta > 90.0 ? (targetAngle - 180.0) : (targetAngle + 180.0);
    }
    return frc::SwerveModuleState{units::velocity::meters_per_second_t{targetSpeed}, {units::degree_t{targetAngle}}};
}

// adapted from team 364s BaseFalconSwerve example
double SwerveModule::PlaceInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = fmod(scopeReference, 360.0);
    if (lowerOffset >= 0) {
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + (360.0 - lowerOffset);
    } else {
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - (360.0 + lowerOffset);
    }
    while (newAngle < lowerBound) {
        newAngle += 360.0;
    }
    while (newAngle > upperBound) {
        newAngle -= 360.0;
    }
    if (newAngle - scopeReference > 180.0) {
        newAngle -= 360.0;
    } else if (newAngle - scopeReference < -180.0) {
        newAngle += 360.0;
    }
    return newAngle;
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {

    // Optimize the reference state to avoid spinning further than 90 degrees*
    const auto state = Optimize(referenceState, {GetTurnEncoderAngle()});
    
    if(usingFalcon) {
        falconTurn->Set(TalonFXControlMode::Position, (double)state.angle.Degrees() / DriveConstants::kTurnEncoderDegreesPerPulse);
    } else {
        neoController->SetReference((double)state.angle.Degrees() / DriveConstants::kTurnEncoderDegreesPerPulse, CANSparkMax::ControlType::kPosition);
    }

    // TalonFX returns velocity per 100ms so the speed needs to be divided by 10
    driveMotor->Set(TalonFXControlMode::Velocity, ((double)state.speed / 10.0) / DriveConstants::kDriveEncoderDistancePerPulse);    
}

// debug
void SwerveModule::SetDrivePower(double power) {
    driveMotor->Set(power);
}

void SwerveModule::SetTurnPower(double power) {
    if(usingFalcon) SetFalconTurnPower(power);
    else SetNeoTurnPower(power);
    // turnMotor->Set(power);
}

void SwerveModule::ResetEncoders() {
    driveMotor->SetSelectedSensorPosition(0);
    if(usingFalcon) falconTurn->SetSelectedSensorPosition(0);
}