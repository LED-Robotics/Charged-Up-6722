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
#include <rev/CANSparkMax.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include "ctre/Phoenix.h"

#include "Constants.h"

using namespace rev;

class SwerveModule {
    public:
        SwerveModule(WPI_TalonFX *drivingMotor, WPI_TalonFX *turningMotor);
        SwerveModule(WPI_TalonFX *drivingMotor, CANSparkMax *turningMotor);
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
        /**
         * Gets the current state of the swerve module.
         *
         * @return a SwerveModuleState representing the module
         */
        frc::SwerveModuleState GetState() const;
        /**
         * Gets the current position of the swerve module.
         *
         * @return a SwerveModulePosition representing the module
         */
        frc::SwerveModulePosition GetPosition() const;
        /**
         * Corrects an angle with a non-continuous range and sets it to 0-360.
         *
         * @return the angle in its corrected range
         */
        static double PlaceInAppropriate0To360Scope(double scopeReference, double newAngle);
        /**
         * Optimize swerve module target to minimize unnecessary movement.
         *
         * @return the optimized SwerveModuleState
         */
        static frc::SwerveModuleState Optimize(const frc::SwerveModuleState& desiredState, frc::Rotation2d currentAngle);
        /**
         * Sets the state of the swerve module.
         */
        void SetDesiredState(const frc::SwerveModuleState& state);
        /**
         * Debug function to set swerve drive motor using power.
         */
        void SetDrivePower(double power);
        /**
         * Debug function to set swerve turn motor using power.
         */
        void SetTurnPower(double power);
        /**
         * Resets the module motors' encoders.
         */
        void ResetEncoders();

    private:

        double GetFalconTurnPosition() const; 
        double GetNeoTurnPosition() const; 

        void SetFalconTurnPower(double power); 
        void SetNeoTurnPower(double power); 

        // motor references
        bool usingFalcon = true;
        WPI_TalonFX *driveMotor;
        WPI_TalonFX *falconTurn;
        CANSparkMax *neoTurn;
};