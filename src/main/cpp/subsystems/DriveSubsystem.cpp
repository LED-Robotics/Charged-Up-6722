// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <iostream>
#include <cmath>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace DriveConstants;
using namespace frc;

DriveSubsystem::DriveSubsystem()
      //Wheel motors
    : backLeft{kBackLeftPort},
      frontLeft{kFrontLeftPort},
      backRight{kBackRightPort},
      frontRight{kFrontRightPort},
      //Degree of wheel motors
      backLeftTheta{kBackLeftThetaPort},
      frontLeftTheta{kFrontLeftThetaPort},
      backRightTheta{kBackRightThetaPort},
      frontRightTheta{kFrontRightThetaPort},

      //Mag encoder motor controllers
      backLeftTalon{kBackLeftTalonPort},
      frontLeftTalon{kFrontLeftTalonPort},
      backRightTalon{kBackRightTalonPort},
      frontRightTalon{kFrontRightTalonPort},

      //Swerve group motors
      s_backLeft{&backLeft, &backLeftTheta},
      s_frontLeft{&frontLeft, &frontLeftTheta},
      s_backRight{&backRight, &backRightTheta},
      s_frontRight{&frontRight, &frontRightTheta},

      //Gryo
      gyro{SPI::Port::kMXP},

      //Odometry
      odometry{kDriveKinematics, {gyro.GetRotation2d()}, {s_frontLeft.GetPosition(), s_frontRight.GetPosition(), s_backLeft.GetPosition(),
      s_backRight.GetPosition()}, frc::Pose2d{}} {

        ResetEncoders();
        ResetOdometry(frc::Pose2d{});
      }

void DriveSubsystem::Periodic() {
  auto blPos = backLeftTalon.GetSensorCollection();
  auto flPos = frontLeftTalon.GetSensorCollection();
  auto brPos = backRightTalon.GetSensorCollection();
  auto frPos = frontRightTalon.GetSensorCollection();
  // Implementation of subsystem periodic method goes here.

  // m_odometry.Update(GetRotation2d(),
  //                   units::meter_t(this->GetLeftEncoderDistance()),
  //                   units::meter_t(this->GetRightEncoderDistance()));
  // Pose2d current = m_odometry.GetPose();
  // std::cout << "Back Left Absolute: " << blPos.GetPulseWidthPosition() << '\n';
  // std::cout << "Back Left Relative: " << backLeftTheta.GetSelectedSensorPosition() * kTurnRatio << '\n';
  // std::cout << "Front Left: " << flPos.GetPulseWidthPosition() << '\n';
  // std::cout << "Back Right: " << brPos.GetPulseWidthPosition() << '\n';
  // std::cout << "Front Right: " << frPos.GetPulseWidthPosition() << '\n';

  odometry.Update(gyro.GetRotation2d(),
                  {s_frontLeft.GetPosition(), s_frontRight.GetPosition(),
                  s_backLeft.GetPosition(), s_backRight.GetPosition()});
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                          units::meters_per_second_t ySpeed,
                          units::degrees_per_second_t rot,
                          bool fieldRelative) {
  auto states = kDriveKinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        xSpeed, ySpeed, rot, gyro.GetRotation2d())
      : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  //Florida, France, Bland, Brazil
  // auto [fl, fr, bl, br] = states;
  std::cout << "X Target: " << (double)xSpeed << '\n';
  std::cout << "Y Target: " << (double)ySpeed << '\n';
  std::cout << "Angle Target: " << (double)rot << '\n';
  SetModuleStates(states);

  // s_backLeft.SetDesiredState(bl);
  // s_frontLeft.SetDesiredState(fl);
  // s_backRight.SetDesiredState(br);
  // s_frontRight.SetDesiredState(fr);
}

void DriveSubsystem::SetModuleStates(
  wpi::array<frc::SwerveModuleState, 4> desiredStates) {
    kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                          AutoConstants::kMaxSpeed);
    std::cout << "FL Speed: " << (double)desiredStates[0].speed << " FL Angle: " << (double)desiredStates[0].angle.Degrees() << '\n';
    s_frontLeft.SetDesiredState(desiredStates[0]);
    std::cout << "FR Speed: " << (double)desiredStates[1].speed << " FR Angle: " << (double)desiredStates[1].angle.Degrees() << '\n';
    s_frontRight.SetDesiredState(desiredStates[1]);
    std::cout << "BL Speed: " << (double)desiredStates[2].speed << " BL Angle: " << (double)desiredStates[2].angle.Degrees() << '\n';
    s_backLeft.SetDesiredState(desiredStates[2]);
    std::cout << "BR Speed: " << (double)desiredStates[3].speed << " BR Angle: " << (double)desiredStates[3].angle.Degrees() << '\n';
    s_backRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::SetDrivePower(double power) {
  s_frontLeft.SetDrivePower(power);
  s_frontRight.SetDrivePower(power);
  s_backLeft.SetDrivePower(power);
  s_backRight.SetDrivePower(power);
}

void DriveSubsystem::SetTurnPower(double power) {
  s_frontLeft.SetTurnPower(power);
  s_frontRight.SetTurnPower(power);
  s_backLeft.SetTurnPower(power);
  s_backRight.SetTurnPower(power);
}

bool DriveSubsystem::ZeroSwervePosition() {
  WPI_TalonFX *turnMotors[4] = {&backLeftTheta, &frontLeftTheta, &backRightTheta, &frontRightTheta};
  WPI_TalonSRX *talons[4] = {&backLeftTalon, &frontLeftTalon, &backRightTalon, &frontRightTalon};
  int poses[4] = {kBLeftMagPos, kFLeftMagPos, kBRightMagPos, kFRightMagPos};
  int zeroed = 0;
  for(int i = 0; i < 4; i++) {
    auto sensor = talons[i]->GetSensorCollection();
    int pos = sensor.GetPulseWidthPosition() / 2;
    double error = ((double)poses[i]) - ((double)pos * kTurnRatio) / 1024;
    double kP = SmartDashboard::GetNumber("homingP", 0.00001);
    double power = 0.05 + (error * kP);

    if(pos < poses[i]) power *= -1;
    std::cout << "Motor" << i + 1 << "Sensor: " << pos << '\n';
    std::cout << "Motor" << i + 1 << "Power: " << power << '\n';
    if((double)pos * kTurnRatio > (double)poses[i] - (double)kZeroDeadzone && (double)pos * kTurnRatio < (double)poses[i] + (double)kZeroDeadzone) {
      turnMotors[i]->Set(0);
      zeroed++;
    } else {
      turnMotors[i]->Set(power);
    }
  }
  if(zeroed == 4) return true;
  else return false;
}

// void DriveSubsystem::Periodic() {
//   // Implementation of subsystem periodic method goes here.
//   // odometry.Update(GetRotation2d(),
//   //                   units::meter_t(this->GetLeftEncoderDistance()),
//   //                   units::meter_t(this->GetRightEncoderDistance()));
//   // Pose2d current = odometry.GetPose();
//   // std::cout << "Coordinates: (" << (double)current.X() << ", " << (double)current.Y() << ")\n";
//   // std::cout << "Rotation: " << (double)current.Rotation().Degrees() << '\n';
// }

// void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
//   // drive.ArcadeDrive(fwd, rot, true);
// }

// void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
//   // leftMotors.SetVoltage(left);
//   // rightMotors.SetVoltage(right);
//   // drive.Feed();
// }

void DriveSubsystem::ResetEncoders() {
  s_frontLeft.ResetEncoders();
  s_backLeft.ResetEncoders();
  s_frontRight.ResetEncoders();
  s_backRight.ResetEncoders();
}

void DriveSubsystem::SetInverted(bool inverted) {
  backLeft.SetInverted(inverted);
  frontLeft.SetInverted(inverted);
  backRight.SetInverted(inverted);
  frontRight.SetInverted(inverted);
}

units::degree_t DriveSubsystem::GetHeading() const {
  return gyro.GetRotation2d().Degrees();
}

void DriveSubsystem::ZeroHeading() {
  gyro.Reset();
}

double DriveSubsystem::GetTurnRate() {
  return -gyro.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  return odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  odometry.ResetPosition(
    GetHeading(),
    {s_frontLeft.GetPosition(), s_frontRight.GetPosition(),
    s_backLeft.GetPosition(), s_backRight.GetPosition()},
    pose);
}