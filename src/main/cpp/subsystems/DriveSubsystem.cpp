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
      gyro{0, "rio"},

      //Odometry
      odometry{kDriveKinematics, {GetRotation()}, {s_frontLeft.GetPosition(), s_frontRight.GetPosition(), s_backLeft.GetPosition(),
      s_backRight.GetPosition()}, frc::Pose2d{{0.0_m, 0.0_m}, {180_deg}}},
      
      xLimiter{kDriveTranslationLimit},
      yLimiter{kDriveTranslationLimit} {
        ResetEncoders();
        ZeroSwervePosition();

        backLeftTheta.SetInverted(true);
        frontLeftTheta.SetInverted(true);
        backRightTheta.SetInverted(true);
        frontRightTheta.SetInverted(true);

        backLeft.SetInverted(false);
        frontLeft.SetInverted(false);
        backRight.SetInverted(false);
        frontRight.SetInverted(false);
        

        // ResetEncoders();
        // ResetOdometry(frc::Pose2d{{0.0_m, 0.0_m}, {180_deg}});
        // ResetOdometry(frc::Pose2d{{0.0_m, 0.0_m}, {90_deg}});
      }

void DriveSubsystem::Periodic() {
  auto blPos = backLeftTalon.GetSensorCollection();
  auto flPos = frontLeftTalon.GetSensorCollection();
  auto brPos = backRightTalon.GetSensorCollection();
  auto frPos = frontRightTalon.GetSensorCollection();
  // Implementation of subsystem periodic method goes here.
  // std::cout << "Back Left Relative: " << backLeftTheta.GetSelectedSensorPosition() << '\n';
  // std::cout << "Back Left" << blPos.GetPulseWidthPosition() << '\n';
  // std::cout << "Front Left" << flPos.GetPulseWidthPosition() << '\n';
  // std::cout << "Back Right" << brPos.GetPulseWidthPosition() << '\n';
  // std::cout << "Front Right" << frPos.GetPulseWidthPosition() << '\n';

  SmartDashboard::PutNumber("Front Left", flPos.GetPulseWidthPosition());
  SmartDashboard::PutNumber("Back Left", blPos.GetPulseWidthPosition());
  SmartDashboard::PutNumber("Front Right", frPos.GetPulseWidthPosition());
  SmartDashboard::PutNumber("Back Right", brPos.GetPulseWidthPosition());

  odometry.Update(GetRotation(),
                  {s_frontLeft.GetPosition(), s_frontRight.GetPosition(),
                  s_backLeft.GetPosition(), s_backRight.GetPosition()});
  // SmartDashboard::PutNumber("odomGyro", (double)odometry.GetPose().Rotation().Degrees());
  SmartDashboard::PutNumber("navxGyro", (double)GetRotation().Degrees());
  // SmartDashboard::PutNumber("turnRate", (double)GetTurnRate());
  auto pose = odometry.GetPose();
  SmartDashboard::PutNumber("poseX", (double)pose.X());
  SmartDashboard::PutNumber("poseY", (double)pose.Y());
  SmartDashboard::PutNumber("poseAngle", (double)pose.Rotation().Degrees());
  SmartDashboard::PutNumber("gyroPitch", GetPitch());
  // SmartDashboard::PutNumber("frontLeftVel", (double)s_frontLeft.GetState().speed);
  // SmartDashboard::PutNumber("frontRightVel", (double)s_frontRight.GetState().speed);

}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                          units::meters_per_second_t ySpeed,
                          units::degrees_per_second_t rot,
                          bool fieldRelative) {
  // arbitrary speed component adjustments
  rot *= 0.5747;
  xSpeed *= -1.0;
  ySpeed *= -1.0;
  if(enableLimiting) {
    xSpeed = xLimiter.Calculate(xSpeed);
    ySpeed = yLimiter.Calculate(ySpeed);
  } else {
  }

  // SmartDashboard::PutNumber("targetXVel", (double)xSpeed);
  // SmartDashboard::PutNumber("targetYVel", (double)ySpeed);
  // SmartDashboard::PutNumber("targetOmega", (double)rot);
  SmartDashboard::PutBoolean("fieldCentric", fieldRelative);
  auto states = kDriveKinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        xSpeed, ySpeed, rot, GetPose().Rotation())
      : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, 5.0_mps);

  SetModuleStates(states);
}

void DriveSubsystem::SetModuleStates(
  wpi::array<frc::SwerveModuleState, 4> desiredStates) {
    kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, 5.0_mps);
    s_frontLeft.SetDesiredState(desiredStates[0]);
    s_frontRight.SetDesiredState(desiredStates[1]);
    s_backLeft.SetDesiredState(desiredStates[2]);
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

void DriveSubsystem::ZeroSwervePosition() {
  WPI_TalonFX *turnMotors[4] = {&backLeftTheta, &frontLeftTheta, &backRightTheta, &frontRightTheta}; // turn motor ref arr
  WPI_TalonSRX *talons[4] = {&backLeftTalon, &frontLeftTalon, &backRightTalon, &frontRightTalon}; // mag encoder TalonSRX ref arr
  double poses[4] = {kBLeftMagPos, kFLeftMagPos, kBRightMagPos, kFRightMagPos}; // arr of correct swerve mag poses
  // iterate through each swerve module and offset theta motors so that they match the absolute mag encoder positions
  for(int i = 0; i < 4; i++) {
    auto sensor = talons[i]->GetSensorCollection();
    double pos = sensor.GetPulseWidthPosition() / 2;  // divid mag pos by 2
    turnMotors[i]->SetSelectedSensorPosition(((poses[i] / 2) / kTurnRatio) - (pos / kTurnRatio));
  }
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
  // return units::degree_t{gyro.GetYaw()};
  return units::degree_t{-gyro.GetAngle()};
}

frc::Rotation2d DriveSubsystem::GetRotation() {
  return GetAngle();
}

void DriveSubsystem::ZeroHeading() {
  // gyro.SetYaw(0.0);
  gyro.Reset();
}

double DriveSubsystem::GetTurnRate() {
  // double data[3] = {0, 0, 0};
  // gyro.GetRawGyro(data);
  // return -data[2];
  return -gyro.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  return odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  backLeft.SetSelectedSensorPosition(0);
  frontLeft.SetSelectedSensorPosition(0);
  backRight.SetSelectedSensorPosition(0);
  frontRight.SetSelectedSensorPosition(0);
  odometry.ResetPosition(
    GetRotation(),
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
  // haven't gotten this to work
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
  // return gyro.GetRoll();
  return 0.0;
}

void DriveSubsystem::SetPoseToHold(frc::Pose2d target) {
  poseToHold = target;
}

frc::Pose2d DriveSubsystem::GetPoseToHold() {
  return poseToHold;
}

void DriveSubsystem::StartHolding() {
  // reset PID controllers
  xHoldController.Reset();
  yHoldController.Reset();
  thetaHoldController.Reset();

  // set PID setpoints to target components
  xHoldController.SetSetpoint((double)poseToHold.X());
  yHoldController.SetSetpoint((double)poseToHold.Y());
  thetaHoldController.SetSetpoint((double)poseToHold.Rotation().Degrees());
}

frc::ChassisSpeeds DriveSubsystem::CalculateHolding() {
  auto current = odometry.GetPose();  // current robot pose
  double angle = (double)current.Rotation().Degrees();  // current robot angle
  double currentAngle = SwerveModule::PlaceInAppropriate0To360Scope(thetaHoldController.GetSetpoint(), angle);  // angle with corrected range
  // SmartDashboard::PutNumber("angleTarget", thetaHoldController.GetSetpoint());
  // SmartDashboard::PutNumber("currentHoldAngle", currentAngle);
  // return ChassisSpeeds needed to hold position correctly
  return frc::ChassisSpeeds{units::meters_per_second_t{xHoldController.Calculate((double)current.X())}, 
  units::meters_per_second_t{yHoldController.Calculate((double)current.Y())}, 
  units::radians_per_second_t{thetaHoldController.Calculate(currentAngle)}};
}