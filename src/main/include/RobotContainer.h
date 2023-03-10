// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include "units/angle.h"

#include "Constants.h"
#include "commands/SetPosition.h"
#include "commands/GyroDock.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/LimelightSubsystem.h"

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

  frc2::Command* GetAutonomousCommand();

  void SetDriveBrakes(bool state);

  void ResetOdometry();

 private:
  // The driver's controller
  // frc::XboxController controller{OIConstants::kDriverControllerPort};
  frc2::CommandXboxController controller{OIConstants::kDriverControllerPort};
  // frc::XboxController controller2{OIConstants::kCoDriverControllerPort};
  frc2::CommandXboxController controller2{OIConstants::kCoDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  
  ElevatorSubsystem elevator;
  
  ArmSubsystem arm;

  IntakeSubsystem intake{&arm};
  bool intakeHold = false;

  frc2::Trigger mainDpadUp{[this]() { return controller.GetPOV() == 0; }};
  frc2::Trigger mainDpadRight{[this]() { return controller.GetPOV() == 90; }};
  frc2::Trigger mainDpadDown{[this]() { return controller.GetPOV() == 180; }};
  frc2::Trigger mainDpadLeft{[this]() { return controller.GetPOV() == 270; }};

  frc2::Trigger partnerDpadUp{[this]() { return controller2.GetPOV() == 0; }};
  frc2::Trigger partnerDpadRight{[this]() { return controller2.GetPOV() == 90; }};
  frc2::Trigger partnerDpadDown{[this]() { return controller2.GetPOV() == 180; }};
  frc2::Trigger partnerDpadLeft{[this]() { return controller2.GetPOV() == 270; }};

  // LimelightSubsystem limelight;

  // frc2::InstantCommand m_driveHalfSpeed{[this] { m_drive.SetMaxOutput(0.5); },
  //                                       {}};
  // frc2::InstantCommand m_driveFullSpeed{[this] { m_drive.SetMaxOutput(1); },
  //                                       {}};
  // frc2::InstantCommand toggleFlywheel{[this] { flywheel.SetFlywheelState(!flywheel.GetFlywheelState()); },
  //                                       {}};
  // frc2::InstantCommand intakeFullPower{[this] { intake.On(); },
  //                                       {}};
  // frc2::InstantCommand intakeOff{[this] { intake.Off(); },
  //                                       {}};
  frc2::Command* GetPositionCommand(int position);

  frc2::Command* HandlePartnerCommands(frc2::Command* solo, frc2::Command* partner);

  frc2::Command* GetEmptyCommand();

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
};