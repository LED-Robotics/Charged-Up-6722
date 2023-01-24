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

    constexpr bool kLeftEncoderReversed = false;
    constexpr bool kRightEncoderReversed = true;

    constexpr auto kTrackwidth = .69_m;

    constexpr int kEncoderResolution = 2048;
    constexpr double kWheelRadius = .0508;
    constexpr double kDriveRatio = 1 / 7.13;
    constexpr double kTurnRatio = 1 / 15.43;
    constexpr double kDriveEncoderDistancePerPulse = (2 * std::numbers::pi * kWheelRadius / kEncoderResolution) * kDriveRatio;
    constexpr double kTurnEncoderDistancePerPulse = (2 * std::numbers::pi / kEncoderResolution) * kTurnRatio;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for *your* robot's drive. The Robot Characterization
    // Toolsuite provides a convenient tool for obtaining these values for your
    // robot.

    constexpr auto ks = 0.60367_V;
    constexpr auto kv = 2.3911 * 1_V * 1_s / 1_m;
    constexpr auto ka = 0.27482 * 1_V * 1_s * 1_s / 1_m;

    // Example value only - as above, this must be tuned for your drive!
    constexpr double kPDriveVel = 3.1116;
}  // namespace DriveConstants

namespace FlywheelConstants {
    constexpr int kMotor1Port = 8;
    constexpr int kOff = 0;
    constexpr int kPowerMode = 1;
    constexpr int kRpmMode = 2;
    constexpr double kDefaultPower = 1.0;

    constexpr double bangBangThreshold = 4.0;
}

namespace IntakeConstants {
    constexpr int kMotorPort = 12;
    constexpr int kOff = 0;
    constexpr int kFullMode = 1;
    constexpr int kPowerMode = 2;
    constexpr double kFullPower = 1.0;
    constexpr double kIntakeDeadzone = 0.1;
}

namespace ElevatorConstants {
    constexpr int kMotor1Port = 11;
    constexpr int kMotor2Port = 10;
    constexpr int kOff = 0;
    constexpr int kOn = 1;
    constexpr double kDefaultPower = 1.0;
}

namespace TurretConstants {
    constexpr int kMotorPort = 9;
    constexpr int kEncoderAPort = 4;
    constexpr int kEncoderBPort = 5;
    constexpr bool kEncoderReversed = true;
    constexpr int kMaxLeft = -2920;
    constexpr int kMaxRight = 3000;
    constexpr double kP = 1.0;
    constexpr double kI = 0.0;
    constexpr int kOff = 0;
    constexpr int kPowerMode = 1;
    constexpr int kPositionMode = 2;
    constexpr double kDefaultPower = 0.6;
}

namespace LiftConstants {
    constexpr int kMotorPort = 6;
    constexpr int kOff = 0;
    constexpr int kForward = 1;
    constexpr int kReverse = 2;
    constexpr int kPowerMode = 3;
    constexpr double kDefaultPower = 1.0;
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