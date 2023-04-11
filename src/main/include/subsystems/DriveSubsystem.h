// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/constraint/SwerveDriveKinematicsConstraint.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/voltage.h>
#include "ctre/Phoenix.h"
#include "AHRS.h"

#include "Constants.h"
#include "SwerveModule.h"

using namespace frc;

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

    /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::degrees_per_second_t rot,
             bool fieldRelative);

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  void SetInverted(bool inverted);

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  void SetDrivePower(double power);

  void SetTurnPower(double power);

  bool ZeroSwervePosition();


  /**
   * Returns the degrees of the robot.
   *
   * @return the robot's degrees, from -180 to 180
   */
  units::degree_t GetAngle() const;

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  frc::Rotation2d GetRotation();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  /**
   * Resets the rate limiter.
   */
  void ResetRateLimiter();

  /**
   * Enable or disable acceleration limiting.
   *
   * @param state Whether or not limiting is enabled.
   */
   void SetLimiting(bool state);

   /**
   * Enable or disable braking.
   *
   * @param state Whether or not braking is enabled.
   */
   void SetBrakeMode(bool state);

   /**
   * Initially configure onboard TalonFX settings for motors.
   */
   void ConfigMotors();

   /**
   * Returns the pitch of the robot.
   *
   * @return the robot's pitch, from -180 to 180
   */
  double GetPitch();

  void driveDistance(units::meter_t distance, units::meters_per_second_t speed);

  void driveDistance(units::meter_t distance);

  void strafeDistance(units::meter_t distance, units::meters_per_second_t speed);

  void strafeDistance(units::meter_t distance);
  
  // void turnToDegrees(units::degree_t angle, units::angular_velocity::degrees_per_second_t speed);


  // frc::Translation2d frontRightLocation{-0.449072_m, -0.449072_m};
  // frc::Translation2d frontLeftLocation{-0.449072_m, 0.449072_m};
  // frc::Translation2d backRightLocation{0.449072_m, -0.449072_m};
  // frc::Translation2d backLeftLocation{0.449072_m, 0.449072_m};

  // frc::Translation2d backLeftLocation{0.449072_m, 0.449072_m};
  // frc::Translation2d backRightLocation{-0.449072_m, 0.449072_m};
  // frc::Translation2d frontLeftLocation{0.449072_m, -0.449072_m};
  // frc::Translation2d frontRightLocation{-0.449072_m, -0.449072_m};

// CURRENT WORKING CONFIG
  // frc::Translation2d frontLeftLocation{-0.449072_m, 0.449072_m};
  // frc::Translation2d frontRightLocation{-0.449072_m, -0.449072_m};
  // frc::Translation2d backLeftLocation{0.449072_m, 0.449072_m};
  // frc::Translation2d backRightLocation{0.449072_m, -0.449072_m};

  frc::Translation2d frontLeftLocation{0.449072_m, 0.449072_m};
  frc::Translation2d frontRightLocation{0.449072_m, -0.449072_m};
  frc::Translation2d backLeftLocation{-0.449072_m, 0.449072_m};
  frc::Translation2d backRightLocation{-0.449072_m, -0.449072_m};

  //Bland, Florida, Brazil, France
  // frc::Translation2d backLeftLocation{-0.449072_m, 0.449072_m};
  // frc::Translation2d frontLeftLocation{0.449072_m, 0.449072_m};
  // frc::Translation2d backRightLocation{-0.449072_m, -0.449072_m};
  // frc::Translation2d frontRightLocation{0.449072_m, -0.449072_m};
  frc::SwerveDriveKinematics<4> kDriveKinematics{frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation};

 private:
 bool enableLimiting = false;
 double initialPitch = 0.0;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  //Wheel motors
  WPI_TalonFX backLeft;
  WPI_TalonFX frontLeft;
  WPI_TalonFX backRight;
  WPI_TalonFX frontRight;
  //Degree of wheel motors
  WPI_TalonFX backLeftTheta;
  WPI_TalonFX frontLeftTheta;
  WPI_TalonFX backRightTheta;
  WPI_TalonFX frontRightTheta;

  //Mag encoder motor controllers
  WPI_TalonSRX backLeftTalon;
  WPI_TalonSRX frontLeftTalon;
  WPI_TalonSRX backRightTalon;
  WPI_TalonSRX frontRightTalon;

  //Swerve motor groups
  SwerveModule s_backLeft;
  SwerveModule s_frontLeft;
  SwerveModule s_backRight;
  SwerveModule s_frontRight;

  
  // The gyro sensor
  AHRS gyro;

  // Odometry class for tracking robot pose
  frc::SwerveDriveOdometry<4> odometry;

  SlewRateLimiter<units::meters_per_second> xLimiter;
  SlewRateLimiter<units::meters_per_second> yLimiter;
};