// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <iostream>
#include <cmath>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/Field2d.h>
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
      gyro{SerialPort::Port::kMXP},

      //Odometry
      odometry{kDriveKinematics, {gyro.GetRotation2d()}, {s_frontLeft.GetPosition(), s_frontRight.GetPosition(), s_backLeft.GetPosition(),
      s_backRight.GetPosition()}, frc::Pose2d{{0.0_m, 0.0_m}, {180_deg}}},
      
      xLimiter{kDriveTranslationLimit},
      yLimiter{kDriveTranslationLimit} {
        ZeroSwervePosition();
        gyro.Reset();
        initialPitch = gyro.GetPitch();
        // gyro.Calibrate();
        // backRightTheta.SetSelectedSensorPosition(0);
        // ConfigMotors();

        // ResetEncoders();
        ResetOdometry(frc::Pose2d{{0.0_m, 0.0_m}, {180_deg}});
      }

void DriveSubsystem::Periodic() {
  // auto blPos = backLeftTalon.GetSensorCollection();
  // auto flPos = frontLeftTalon.GetSensorCollection();
  // auto brPos = backRightTalon.GetSensorCollection();
  // auto frPos = frontRightTalon.GetSensorCollection();
  // Implementation of subsystem periodic method goes here.
  // std::cout << "Back Left Absolute: " << blPos.GetPulseWidthPosition() << '\n';
  // std::cout << "Back Left Relative: " << backLeftTheta.GetSelectedSensorPosition() << '\n';
  // std::cout << "Front Left: " << flPos.GetPulseWidthPosition() << '\n';
  // std::cout << "Back Right: " << brPos.GetPulseWidthPosition() << '\n';
  // std::cout << "Front Right: " << frPos.GetPulseWidthPosition() << '\n';



  odometry.Update(gyro.GetRotation2d(),
                  {s_frontLeft.GetPosition(), s_frontRight.GetPosition(),
                  s_backLeft.GetPosition(), s_backRight.GetPosition()});
  // SmartDashboard::PutNumber("odomGyro", (double)odometry.GetPose().Rotation().Degrees());
  // SmartDashboard::PutNumber("navxGyro", (double)GetAngle());
  auto pose = odometry.GetPose();
  SmartDashboard::PutNumber("poseX", (double)pose.X());
  SmartDashboard::PutNumber("poseY", (double)pose.Y());
  SmartDashboard::PutNumber("poseAngle", (double)pose.Rotation().Degrees());
  // SmartDashboard::PutNumber("gyroPitch", gyro.GetPitch());
  // SmartDashboard::PutNumber("frontLeftVel", (double)s_frontLeft.GetState().speed);
  // SmartDashboard::PutNumber("frontRightVel", (double)s_frontRight.GetState().speed);

}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                          units::meters_per_second_t ySpeed,
                          units::degrees_per_second_t rot,
                          bool fieldRelative) {
  if(enableLimiting) {
    // std::cout << "Slew Rate Limiter Is On!\n";
    xSpeed = xLimiter.Calculate(xSpeed);
    ySpeed = yLimiter.Calculate(ySpeed);
  } else {
    // std::cout << "Slew Rate Limiter Is Off!\n";
  }

  // SmartDashboard::PutNumber("targetXVel", (double)xSpeed);
  // SmartDashboard::PutNumber("targetYVel", (double)ySpeed);
  SmartDashboard::PutNumber("fieldCentric", fieldRelative);
  auto states = kDriveKinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        xSpeed, ySpeed, rot, GetPose().Rotation() * -1)
      : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  // std::cout << "X Target: " << (double)xSpeed << '\n';
  // std::cout << "Y Target: " << (double)ySpeed << '\n';
  // std::cout << "Angle Target: " << (double)rot << '\n';
  SetModuleStates(states);
}

void DriveSubsystem::SetModuleStates(
  wpi::array<frc::SwerveModuleState, 4> desiredStates) {
    kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                          AutoConstants::kMaxSpeed);
    // std::cout << "FL Speed: " << (double)desiredStates[0].speed << " FL Angle: " << (double)desiredStates[0].angle.Degrees() << '\n';
    s_frontLeft.SetDesiredState(desiredStates[0]);
    // std::cout << "FR Speed: " << (double)desiredStates[1].speed << " FR Angle: " << (double)desiredStates[1].angle.Degrees() << '\n';
    s_frontRight.SetDesiredState(desiredStates[1]);
    // std::cout << "BL Speed: " << (double)desiredStates[2].speed << " BL Angle: " << (double)desiredStates[2].angle.Degrees() << '\n';
    s_backLeft.SetDesiredState(desiredStates[2]);
    // std::cout << "BR Speed: " << (double)desiredStates[3].speed << " BR Angle: " << (double)desiredStates[3].angle.Degrees() << '\n';
    s_backRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::SetDrivePower(double power) {
  // std::cout << "Power: " << power << '\n';
  // std::cout << "Velocity: " << backLeft.GetSelectedSensorVelocity() << '\n';
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
  double poses[4] = {kBLeftMagPos, kFLeftMagPos, kBRightMagPos, kFRightMagPos};
  for(int i = 0; i < 4; i++) {
    auto sensor = talons[i]->GetSensorCollection();
    double pos = sensor.GetPulseWidthPosition() / 2;
    turnMotors[i]->SetSelectedSensorPosition(((poses[i] / 2) / kTurnRatio) - (pos / kTurnRatio));
  }
  return true;
}

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

units::degree_t DriveSubsystem::GetAngle() const {
  return units::degree_t{gyro.GetAngle()};
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
    gyro.GetRotation2d(),
    {s_frontLeft.GetPosition(), s_frontRight.GetPosition(),
    s_backLeft.GetPosition(), s_backRight.GetPosition()},
    pose);
}

void DriveSubsystem::SetLimiting(bool state) {
  enableLimiting = state;
}

void DriveSubsystem::SetBrakeMode(bool state) {
  ctre::phoenix::motorcontrol::NeutralMode mode;
  if(state) mode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
  else mode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
  backLeft.SetNeutralMode(mode);
  frontLeft.SetNeutralMode(mode);
  backRight.SetNeutralMode(mode);
  frontRight.SetNeutralMode(mode);
  backLeftTheta.SetNeutralMode(mode);
  frontLeftTheta.SetNeutralMode(mode);
  backRightTheta.SetNeutralMode(mode);
  frontRightTheta.SetNeutralMode(mode);
}

void DriveSubsystem::ConfigMotors() {
  backLeft.Config_kP(0, driveKp, 100);
  frontLeft.Config_kP(0, driveKp, 100);
  backRight.Config_kP(0, driveKp, 100);
  frontRight.Config_kP(0, driveKp, 100);

  backLeft.Config_kF(0, driveKf, 100);
  frontLeft.Config_kF(0, driveKf, 100);
  backRight.Config_kF(0, driveKf, 100);
  frontRight.Config_kF(0, driveKf, 100);

  backLeftTheta.Config_kP(0, turnKp, 100);
  frontLeftTheta.Config_kP(0, turnKp, 100);
  backRightTheta.Config_kP(0, turnKp, 100);
  frontRightTheta.Config_kP(0, turnKp, 100);
}

double DriveSubsystem::GetPitch() {
  return gyro.GetPitch();
}