// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <units/voltage.h>
#include "ctre/Phoenix.h"
#include "AHRS.h"
#include <frc/kinematics/ChassisSpeeds.h>

#include "Constants.h"

using namespace frc;
using namespace frc2;

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  void ArcadeDrive(double fwd, double rot);

  /**
   * Controls each side of the drive directly with a voltage.
   *
   * @param left the commanded left output
   * @param right the commanded right output
   */
  void TankDriveVolts(units::volt_t left, units::volt_t right);

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Resets the NavX to currently read a position of 0.
   */
  void ResetGyro();

  /**
   * Gets the distance of the left encoder.
   *
   * @return the left encoder distance
   */
  double GetLeftEncoderDistance();
    
   /**
   * Gets the distance of the right encoder.
   *
   * @return the right encoder distance
   */
  double GetRightEncoderDistance();

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  double GetAverageEncoderDistance();

  /**
   * Gets the rate of the left encoder.
   *
   * @return the left encoder distance
   */
  double GetLeftEncoderRate();
    
   /**
   * Gets the rate of the right encoder.
   *
   * @return the right encoder distance
   */
  double GetRightEncoderRate();

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  frc::Encoder& GetLeftEncoder();

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  frc::Encoder& GetRightEncoder();

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive
   * more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  void SetMaxOutput(double maxOutput);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  units::degree_t GetHeading() const;

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  /**
   * Returns the Rotation2d object from the angle of the gyro.
   *
   * @return The Rotation2d of the gyro.
   */
  Rotation2d GetRotation2d();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  frc::ChassisSpeeds GetWheelSpeeds();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  // WPI_VictorSPX backLeft;
  // WPI_VictorSPX frontLeft;
  // WPI_VictorSPX backRight;
  // WPI_VictorSPX frontRight;

  WPI_TalonFX backLeft;
  WPI_TalonFX frontLeft;
  WPI_TalonFX backRight;
  WPI_TalonFX frontRight;

  WPI_TalonFX backLeftTheta;
  WPI_TalonFX frontLeftTheta;
  WPI_TalonFX backRightTheta;
  WPI_TalonFX frontRightTheta;

  // The motors on the left side of the drive
  frc::MotorControllerGroup leftMotors{backLeft, frontLeft};

  // The motors on the right side of the drive
  frc::MotorControllerGroup rightMotors{backRight, frontRight};

  // The robot's drive
  frc::DifferentialDrive drive{leftMotors, rightMotors};

  // The gyro sensor
  AHRS gyro;

  // Odometry class for tracking robot pose
  frc::SwerveDriveOdometry<4> odometry;
};
