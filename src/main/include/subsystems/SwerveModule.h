// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include <frc/Encoder.h>
#include <frc/geometry/Pose2d.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include "ctre/Phoenix.h"

#include "Constants.h"

class SwerveModule {
    public:
        SwerveModule(WPI_TalonFX *drivingMotor, WPI_TalonFX *turningMotor);
        /**
         * Gets the distance of the drive encoder.
         *
         * @return the drive encoder distance
         */
        units::meter_t GetDriveEncoderDistance() const;

        /**
         * Gets the distance of the turn encoder.
         *
         * @return the turn encoder distance
         */
        units::degree_t GetTurnEncoderAngle() const;

        /**
         * Gets the rate of the encoder.
         *
         * @return the encoder distance
         */
        units::meters_per_second_t GetDriveEncoderRate() const;
        frc::SwerveModuleState GetState() const;
        frc::SwerveModulePosition GetPosition() const;
        void SetDesiredState(const frc::SwerveModuleState& state);
        void SetDrivePower(double power);
        void SetTurnPower(double power);
        void ResetEncoders();

    private:
  

        static constexpr auto kModuleMaxAngularVelocity =
            std::numbers::pi * 57.2958_deg_per_s;  // radians per second
        static constexpr auto kModuleMaxAngularAcceleration =
            std::numbers::pi * 114.592_deg_per_s / 1_s;  // radians per second^2

        WPI_TalonFX *driveMotor;
        WPI_TalonFX *turnMotor;

        frc2::PIDController drivePIDController{DriveConstants::kPDriveVel, 0, 0};
        // frc2::PIDController drivePIDController{1.0, 0, 0};
        frc::ProfiledPIDController<units::degrees> turningPIDController{
            DriveConstants::kPTurnVel,
            0.0,
            0.0,
            {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

        frc::SimpleMotorFeedforward<units::meters> driveFeedforward{DriveConstants::driveKs, DriveConstants::driveKv, DriveConstants::driveKa};
        frc::SimpleMotorFeedforward<units::degrees> turnFeedforward{DriveConstants::turnKs, DriveConstants::turnKv, DriveConstants::turnKa};
};