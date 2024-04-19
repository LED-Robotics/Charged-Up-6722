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
#include <ctre/Phoenix.h>

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

    constexpr int kBLeftMagPos = -3491;
    constexpr int kFLeftMagPos = 2117;
    constexpr int kBRightMagPos = 8261;
    constexpr int kFRightMagPos = -3397;
    
    // info for odometry
    constexpr int kEncoderResolution = 2048;
    constexpr double kWheelRadius = .0508;
    constexpr double kDriveRatio = 1 / 5.5;
    constexpr double kTurnRatio = 1 / 10.29;
    constexpr double kDriveEncoderDistancePerPulse = (2 * std::numbers::pi * kWheelRadius / (double)kEncoderResolution) * kDriveRatio;
    constexpr double kTurnEncoderDegreesPerPulse = (360.0 / (double)kEncoderResolution) * kTurnRatio;

    // TalonFX config
    constexpr auto driveKf = 0.05614;
    constexpr auto driveKp = 0.1;

    constexpr auto turnKp = 0.275;

    // drivetrain controller constants
    constexpr double kDriveDeadzone = 0.05;
    constexpr double kDriveCurveExtent = 0.6;
    constexpr auto kDriveTranslationLimit = 6.0_mps_sq;
}  // namespace DriveConstants

namespace ElevatorConstants {
    // ports
    constexpr int kLeftMotorPort = 12;
    constexpr int kRightMotorPort = 13;
    // states
    constexpr int kOff = 0;
    constexpr int kPowerMode = 1;
    constexpr int kPositionMode = 2;
    // default power in power mode
    constexpr double kDefaultPower = 1.0;
    // elevator position deadzone
    constexpr double kElevatorDeadzone = 500;
    // elevator position constants
    constexpr int kStartPosition = 10;
    constexpr int kFloorPickupPosition = 100;
    constexpr int kFloorStandingPickupPosition = 100;
    constexpr int kMidDropoffPosition = 22000;
    constexpr int kHighDropoffPosition = 90000;
    constexpr int kDoubleStation = 40000;   // vertical cone
    // constexpr int kDoubleStation = 24000;    // laying down cone
    constexpr int kCubePosition = 600;
    constexpr int kPositionDeadzone = 3000;
    // TalonFX config
    constexpr int kP = 0.1;
}

namespace ArmConstants {
    // ports
    constexpr int kLeftMotorPort = 14;
    constexpr int kRightMotorPort = 15;
    // states
    constexpr int kOff = 0;
    constexpr int kPowerMode = 1;
    constexpr int kPositionMode = 2;
    constexpr int kAngleMode = 3;
    // default power in power mode
    constexpr double kDefaultPower = 1.0;
    // arm min/max positions
    constexpr int kArmDegreeMin = 0;   // encoder counts at the arm's minimum usable position
    constexpr int kMinAngleCounts = -200;   // encoder counts at the arm's minimum usable position
    constexpr int kMaxAngleCounts = 4000;   // encoder counts at the arm's max usable position
    constexpr double kArmDegreeMax = 200; // degrees the arm is capable of moving 
    // for arm feed forward trig
    constexpr int kCountsPerDegree = 545;
    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.06;
    // arm position constants
    constexpr double kStartAngle = 8.26;
    constexpr double kFloorPickupAngle = 18.5;
    constexpr double kFloorStandingPickupAngle = 50.0;
    constexpr double kMidDropoffAngle = 136.7;
    constexpr double kMidCubeAngle = 92.0;
    constexpr double kHighDropoffAngle = 136.7;
    constexpr double kHighCubeAngle = 135.0;
    constexpr double kDoubleStation = 136.7;
    constexpr double kAutonStart = 275.0;
    // position deadzone
    constexpr int kPositionDeadzone = 15000;
    // TalonFX config
    constexpr int kP = 0.012;
}

namespace IntakeConstants {
    // ports
    constexpr int kIntakePort = 16;
    constexpr int kWristPort = 17;
    // states
    constexpr int kOff = 0;
    constexpr int kFullMode = 1;
    constexpr int kPowerMode = 2;
    constexpr int kPositionMode = 0;
    constexpr int kAngleMode = 1;
    // power in full power mode
    constexpr double kFullPower = 1.0;
    // current limit
    constexpr double kCurrentLimit = 15.0;
    // power deadzone
    constexpr double kIntakeDeadzone = 0.1;
    // wrist encoder to angle constant
    constexpr int kCountsPerDegree = 143;
    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = -0.07;
    // wrist position constants
    constexpr double kStartAngle = -30.0;
    constexpr int kStartPosition = 5500;
    constexpr int kFloorPickupPosition = 22700;
    constexpr int kFloorStandingPickupPosition = 25417;
    constexpr int kMidCubePosition = 23750;
    constexpr int kMidDropoffPosition = 40713;
    constexpr int kHighCubePosition = 32400;
    constexpr int kHighDropoffPosition = 37900;
    constexpr int kDoubleStation = 37900;
    constexpr int kAutonStart = 53900;
    // wrist position deadzone
    constexpr int kPositionDeadzone = 6666;
    // TalonFX config
    constexpr int kP = 0.03;
}

namespace LimelightConstants {
    // states
    constexpr int kOff = 0;
    constexpr int kOn = 1;
    constexpr int kBlink = 2;
    constexpr int kUsePipeline = 3;
}

namespace AutoConstants {
        // for PID/pathfinding
        constexpr auto kMaxSpeed = 2.5_mps;
        constexpr auto kMaxAcceleration = 2_mps_sq;
        constexpr auto kAngularSpeed = 180_deg_per_s;
        constexpr auto kMaxAngularAcceleration = 180_deg_per_s_sq;

        constexpr double kPXController = 0.4;
        constexpr double kPYController = 0.4;
        constexpr double kPThetaController = 0.0;
} //namespace AutoConstants

namespace OIConstants {
    constexpr int kDriverControllerPort = 0;
    constexpr int kCoDriverControllerPort = 1;
}  // namespace OIConstants

namespace BlinkinConstants {
    constexpr int kBlinkinPort = 0;
}   //namespace BlinkinConstants