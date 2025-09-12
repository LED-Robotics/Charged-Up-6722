// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/DriverStation.h>
#include <frc2/command/button/Trigger.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Commands.h>
#include <frc2/command/Command.h>
#include <frc2/command/RepeatCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include "subsystems/DriveSubsystem/DriveSubsystem.h"
#include "subsystems/TelescopeSubsystem/TelescopeSubsystem.h"
#include "subsystems/ArmSubsystem/ArmSubsystem.h"
#include "subsystems/WristSubsystem/WristSubsystem.h"
#include "subsystems/IntakeSubsystem/IntakeSubsystem.h"
#include "units/angle.h"

#include "GlobalConstants.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

class RobotContainer {
 public:
  RobotContainer();

  struct KinematicsPose {
    units::length::meter_t telescopePose;
    units::angle::degree_t armAngle;
    units::angle::degree_t wristAngle;
  };

  frc2::Command* GetAutonomousCommand();
  /**
   * Enable odometry updates from AprilTag tracking.
   */  
  void EnableTagTracking();
  /**
   * Disable odometry updates from AprilTag tracking.
   */  
  void DisableTagTracking();
  /**
   * Set the brake mode of most robot motors.
   */  
  void SetDriveBrakes(bool state);
  /**
   * Function to handle the IntakeSubsystem's control logic.
   */  
  void HandleIntake();
  /**
   * Set the state of the DriveSubsystem's SlewRateLimiters.
   * 
   * @param state subsystem state
   */  
  void SetSlew(bool state);
  /**
   * Function to return if you are on blue alliance.
   * 
   * @return bool for blue alliance
   */
  bool IsBlue();

 private:

  // Global Flags
  bool fieldCentric = true;

  bool intakeHold = false;

  // The driver's controller
  frc2::CommandXboxController controller{OIConstants::kDriverControllerPort};
  // The partner controller
  frc2::CommandXboxController controller2{OIConstants::kCoDriverControllerPort};

  frc2::Trigger mainDpadUp{controller.POV(0)};
  frc2::Trigger mainDpadDown{controller.POV(180)};
  frc2::Trigger mainDpadLeft{controller.POV(270)};
  frc2::Trigger mainDpadRight{controller.POV(90)};

  frc2::Trigger mainDpadUp2{controller2.POV(0)};
  frc2::Trigger mainDpadDown2{controller2.POV(180)};
  frc2::Trigger mainDpadLeft2{controller2.POV(270)};
  frc2::Trigger mainDpadRight2{controller2.POV(90)};

  DriveSubsystem drive{};

  TelescopeSubsystem telescope{};

  ArmSubsystem arm{};

  WristSubsystem wrist{};

  IntakeSubsystem intake{};

  KinematicsPose startingPose{0.0_m, 20_deg, -31.0_deg};
  KinematicsPose floorPose{0.0_m, 31.5_deg, -144.0_deg};
  KinematicsPose floorStandingPose{0.0_m, 31.5_deg, -112.0_deg};
  KinematicsPose middlePose{0.0_m, 142.8_deg, -270.0_deg};
  KinematicsPose topPose{1.1_m, 135.8_deg, -255.16_deg};

  /**
   * Return the command pointer that sets all subsystem kinematics.
   */
  frc2::CommandPtr SetAllKinematics(KinematicsPose pose);
};
