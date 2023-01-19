// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <iostream>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

using namespace DriveConstants;
using namespace frc;

DriveSubsystem::DriveSubsystem()
    : backLeft{kBackLeftPort},
      frontLeft{kFrontLeftPort},
      backRight{kBackRightPort},
      frontRight{kFrontRightPort},
      backLeftTheta{kBackLeftThetaPort},
      frontLeftTheta{kFrontLeftThetaPort},
      backRightTheta{kBackRightThetaPort},
      frontRightTheta{kFrontRightThetaPort},

      gyro{SPI::Port::kMXP},
      odometry{kDriveKinematics, {units::degree_t{gyro.GetAngle()}}},
  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  // rightMotors.SetInverted(true);

  // Set the distance per pulse for the encoders

  ResetEncoders();
  ResetGyro();
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  // odometry.Update(GetRotation2d(),
  //                   units::meter_t(this->GetLeftEncoderDistance()),
  //                   units::meter_t(this->GetRightEncoderDistance()));
  // Pose2d current = odometry.GetPose();
  // std::cout << "Coordinates: (" << (double)current.X() << ", " << (double)current.Y() << ")\n";
  // std::cout << "Rotation: " << (double)current.Rotation().Degrees() << '\n';
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  // drive.ArcadeDrive(fwd, rot, true);
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
  // leftMotors.SetVoltage(left);
  // rightMotors.SetVoltage(right);
  // drive.Feed();
}

void DriveSubsystem::ResetEncoders() {
  // backLeft.SetSelectedSensorPosition(0);
  // backRight.SetSelectedSensorPosition(0);
}

void DriveSubsystem::ResetGyro() {
  // gyro.Reset();
}

double DriveSubsystem::GetLeftEncoderDistance() {
  // return backLeft.GetSelectedSensorPosition() * kEncoderDistancePerPulse * (kLeftEncoderReversed ? -1.0 : 1.0);
}

double DriveSubsystem::GetRightEncoderDistance() {
  // return backRight.GetSelectedSensorPosition() * kEncoderDistancePerPulse * (kRightEncoderReversed ? -1.0 : 1.0);
}

double DriveSubsystem::GetLeftEncoderRate() {
  // return backLeft.GetSelectedSensorVelocity() * 10 * kEncoderDistancePerPulse;
}

double DriveSubsystem::GetRightEncoderRate() {
  // return backRight.GetSelectedSensorVelocity() * 10 * kEncoderDistancePerPulse;
}

double DriveSubsystem::GetAverageEncoderDistance() {
  // return (this->GetLeftEncoderDistance() + this->GetRightEncoderDistance()) / 2.0;
}

void DriveSubsystem::SetMaxOutput(double maxOutput) {
  // drive.SetMaxOutput(maxOutput);
}

units::degree_t DriveSubsystem::GetHeading() const {
  // return units::degree_t{-gyro.GetAngle()};
}

double DriveSubsystem::GetTurnRate() {
  // return -gyro.GetRate();
}

Rotation2d DriveSubsystem::GetRotation2d() {
  // return Rotation2d{units::degree_t{-gyro.GetAngle()}};
}

frc::Pose2d DriveSubsystem::GetPose() {
  // return odometry.GetPose();
}

frc::ChassisSpeeds DriveSubsystem::GetWheelSpeeds() {
  // return {units::meters_per_second_t(this->GetLeftEncoderRate()),
  //         units::meters_per_second_t(this->GetRightEncoderRate())};
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  // ResetEncoders();
  // ResetGyro();
  // odometry.ResetPosition({units::degree_t{-gyro.GetAngle()}}, units::meter_t(this->GetLeftEncoderDistance()), units::meter_t(this->GetRightEncoderDistance()), pose);
}
