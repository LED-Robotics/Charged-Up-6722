// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <iostream>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

using namespace DriveConstants;
using namespace frc;

DriveSubsystem::DriveSubsystem()
    : m_left1{kLeftMotor1Port},
      m_left2{kLeftMotor2Port},
      m_right1{kRightMotor1Port},
      m_right2{kRightMotor2Port},
      m_gyro{SPI::Port::kMXP},
      m_odometry{{units::degree_t{m_gyro.GetAngle()}}, 0_m, 0_m} {
  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  m_rightMotors.SetInverted(true);

  // Set the distance per pulse for the encoders

  ResetEncoders();
  ResetGyro();
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(GetRotation2d(),
                    units::meter_t(this->GetLeftEncoderDistance()),
                    units::meter_t(this->GetRightEncoderDistance()));
  Pose2d current = m_odometry.GetPose();
  std::cout << "Coordinates: (" << (double)current.X() << ", " << (double)current.Y() << ")\n";
  std::cout << "Rotation: " << (double)current.Rotation().Degrees() << '\n';
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot, true);
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotors.SetVoltage(left);
  m_rightMotors.SetVoltage(right);
  m_drive.Feed();
}

void DriveSubsystem::ResetEncoders() {
  m_left1.SetSelectedSensorPosition(0);
  m_right1.SetSelectedSensorPosition(0);
}

void DriveSubsystem::ResetGyro() {
  m_gyro.Reset();
}

double DriveSubsystem::GetLeftEncoderDistance() {
  return m_left1.GetSelectedSensorPosition() * kEncoderDistancePerPulse * (kLeftEncoderReversed ? -1.0 : 1.0);
}

double DriveSubsystem::GetRightEncoderDistance() {
  return m_right1.GetSelectedSensorPosition() * kEncoderDistancePerPulse * (kRightEncoderReversed ? -1.0 : 1.0);
}

double DriveSubsystem::GetLeftEncoderRate() {
  return m_left1.GetSelectedSensorVelocity() * 10 * kEncoderDistancePerPulse;
}

double DriveSubsystem::GetRightEncoderRate() {
  return m_right1.GetSelectedSensorVelocity() * 10 * kEncoderDistancePerPulse;
}

double DriveSubsystem::GetAverageEncoderDistance() {
  return (this->GetLeftEncoderDistance() + this->GetRightEncoderDistance()) / 2.0;
}

void DriveSubsystem::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

units::degree_t DriveSubsystem::GetHeading() const {
  return units::degree_t{-m_gyro.GetAngle()};
}

double DriveSubsystem::GetTurnRate() {
  return -m_gyro.GetRate();
}

Rotation2d DriveSubsystem::GetRotation2d() {
  return Rotation2d{units::degree_t{-m_gyro.GetAngle()}};
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
  return {units::meters_per_second_t(this->GetLeftEncoderRate()),
          units::meters_per_second_t(this->GetRightEncoderRate())};
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  ResetGyro();
  m_odometry.ResetPosition({units::degree_t{-m_gyro.GetAngle()}}, units::meter_t(this->GetLeftEncoderDistance()), units::meter_t(this->GetRightEncoderDistance()), pose);
}
