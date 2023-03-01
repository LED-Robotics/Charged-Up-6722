// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <numbers>
#include <frc/Encoder.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <units/angle.h>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
    //Wheel motors
    constexpr int kBackLeftPort = 0;
    constexpr int kFrontLeftPort = 1;
    constexpr int kBackRightPort = 2;
    constexpr int kFrontRightPort = 3;
    //Degree of wheel motors
    constexpr int kBackLeftThetaPort = 4;
    constexpr int kFrontLeftThetaPort = 5;
    constexpr int kBackRightThetaPort = 6;
    constexpr int kFrontRightThetaPort = 7;

    constexpr int kBackLeftTalonPort = 8;
    constexpr int kFrontLeftTalonPort = 9;
    constexpr int kBackRightTalonPort = 10;
    constexpr int kFrontRightTalonPort = 11;

    constexpr int kBLeftMagPos = 2496;
    constexpr int kFLeftMagPos = 3582;
    constexpr int kBRightMagPos = 2836;
    constexpr int kFRightMagPos = 1061;

    constexpr int kZeroDeadzone = 100;

    constexpr bool kLeftEncoderReversed = false;
    constexpr bool kRightEncoderReversed = true;

    constexpr auto kTrackwidth = .69_m;

    constexpr int kEncoderResolution = 2048;
    constexpr double kWheelRadius = .0508;
    constexpr double kDriveRatio = 1 / 6.55;
    constexpr double kTurnRatio = 1 / 10.29;
    constexpr double kDriveEncoderDistancePerPulse = (2 * std::numbers::pi * kWheelRadius / (double)kEncoderResolution) * kDriveRatio;
    constexpr double kTurnEncoderDegreesPerPulse = (360.0 / (double)kEncoderResolution) * kTurnRatio;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for *your* robot's drive. The Robot Characterization
    // Toolsuite provides a convenient tool for obtaining these values for your
    // robot.

    constexpr auto driveKs = 0.2256_V;
    constexpr auto driveKv = 2.2285 * 1_V * 1_s / 1_m;
    constexpr auto driveKa = 0.19337 * 1_V * 1_s * 1_s / 1_m;

    constexpr auto turnKs = 0.257951_V;
    constexpr auto turnKv = 0.0020769 * 1_V * 1_s / 1_deg;
    constexpr auto turnKa = 6.1723E-05 * 1_V * 1_s * 1_s / 1_deg;

    // Example value only - as above, this must be tuned for your drive!
    constexpr double kPDriveVel = 1.0;
    constexpr double kPTurnVel = 0.001;
    // constexpr double kPTurnVel = 2.1916E-06;
}  // namespace DriveConstants

namespace ElevatorConstants {
    constexpr int kLeftMotorPort = 12;
    constexpr int kRightMotorPort = 13;
    constexpr int kLeftStopPort = 0;
    constexpr int kRightStopPort = 1;
    constexpr int kOff = 0;
    constexpr int kPowerMode = 1;
    constexpr int kPositionMode = 2;
    constexpr double kDefaultPower = 1.0;
    constexpr double kElevatorDeadzone = 500;
}

namespace ArmConstants {
    constexpr int kLeftMotorPort = 14;
    constexpr int kRightMotorPort = 15;
    constexpr int kOff = 0;
    constexpr int kPowerMode = 1;
    constexpr int kPositionMode = 2;
    constexpr double kDefaultPower = 1.0;
    constexpr int kArmDegreeMin = 0;   // encoder counts at the arm's minimum usable position
    constexpr int kMinAngleCounts = -200;   // encoder counts at the arm's minimum usable position
    constexpr int kMaxAngleCounts = 4000;   // encoder counts at the arm's max usable position
    constexpr double kArmDegreeMax = 200; // degrees the arm is capable of moving 
}

namespace IntakeConstants {
    constexpr int kIntakePort = 16;
    constexpr int kWristPort = 17;
    constexpr int kOff = 0;
    constexpr int kFullMode = 1;
    constexpr int kPowerMode = 2;
    constexpr double kFullPower = 1.0;
    constexpr double kCurrentLimit = 0.5;
    constexpr double kIntakeDeadzone = 0.1;
}

namespace LimelightConstants {
    constexpr int kOff = 0;
    constexpr int kOn = 1;
    constexpr int kBlink = 2;
    constexpr int kUsePipeline = 3;
}

namespace AutoConstants {
        constexpr auto kMaxSpeed = 3_mps;
        constexpr auto kMaxAcceleration = 3_mps_sq;
        constexpr auto kAngularSpeed = 180_deg_per_s;
        constexpr auto kMaxAngularAcceleration = 180_deg_per_s_sq;

        constexpr double kPXController = .5;
        constexpr double kPYController = .5;
        constexpr double kPThetaController = .5;

        extern const frc::TrapezoidProfile<units::radians>::Constraints
            kThetaControllerConstraints;
} //namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kCoDriverControllerPort = 1;
}  // namespace OIConstants