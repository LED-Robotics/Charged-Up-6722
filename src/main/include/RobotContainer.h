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
#include <ctre\Phoenix.h>

#include "Constants.h"
#include "commands/SetPosition.h"
#include "commands/GyroDock.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/LimelightSubsystem.h"
#include "iostream"
#include "frc/motorcontrol/Spark.h"

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
 
  void HandleIntake();
  
  void SetSlew(bool state);

  void SetDriveReversed(bool reversed);

 private:
  // The driver's controller
  // frc::XboxController controller{OIConstants::kDriverControllerPort};
  frc2::CommandXboxController controller{OIConstants::kDriverControllerPort};
  // frc::XboxController controller2{OIConstants::kCoDriverControllerPort};
  frc2::CommandXboxController controller2{OIConstants::kCoDriverControllerPort};

  //Blinkin
  Spark blinkin{BlinkinConstants::kBlinkinPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  
  ElevatorSubsystem elevator;
  
  ArmSubsystem arm;

  IntakeSubsystem intake{&arm};
  
  bool intakeHold = false;

  bool fieldCentric = false;

  frc2::Trigger mainDpadUp{[this]() { return controller.GetPOV() == 0; }};
  frc2::Trigger mainDpadRight{[this]() { return controller.GetPOV() == 90; }};
  frc2::Trigger mainDpadDown{[this]() { return controller.GetPOV() == 180; }};
  frc2::Trigger mainDpadLeft{[this]() { return controller.GetPOV() == 270; }};

  frc2::Trigger partnerDpadUp{[this]() { return controller2.GetPOV() == 0; }};
  frc2::Trigger partnerDpadRight{[this]() { return controller2.GetPOV() == 90; }};
  frc2::Trigger partnerDpadDown{[this]() { return controller2.GetPOV() == 180; }};
  frc2::Trigger partnerDpadLeft{[this]() { return controller2.GetPOV() == 270; }};

  // LimelightSubsystem limelight;

  frc2::RunCommand verticalPickup{
      [this] {
        double elevatorY = controller2.GetLeftY();
        if(elevatorY > 0.0) elevatorY = 0.0;
        elevator.SetTargetPosition(15000 + (-elevatorY * 75000));
        double armY = controller2.GetRightY();
        if(armY < 0.0) armY = 0.0;
        arm.SetTargetAngle(136.0 - (armY * 106.0));
        HandleIntake();
        intake.SetWristState(IntakeConstants::kAngleMode);
        double wristAdjust = controller2.GetRightTriggerAxis() * 90.0;
        intake.SetTargetAngle(200 - wristAdjust);
      },
      {&elevator, &arm, &intake}};


  frc2::InstantCommand toggleFieldCentric{[this] { fieldCentric = !fieldCentric; },
                                        {}};

  frc2::InstantCommand rumblePrimaryOn{[this] { controller.SetRumble(GenericHID::kBothRumble, 1.0); },
                                        {}};

  frc2::InstantCommand rumbleSecondaryOn{[this] { controller2.SetRumble(GenericHID::kBothRumble, 1.0); },
                                        {}};
  
  frc2::InstantCommand rumblePrimaryOff{[this] { controller.SetRumble(GenericHID::kBothRumble, 0.0); },
                                        {}};

  frc2::InstantCommand rumbleSecondaryOff{[this] { controller2.SetRumble(GenericHID::kBothRumble, 0.0); },
                                        {}};
  
  
  frc2::InstantCommand SetBlinkinAButton{[this] { blinkin.Set(.41); /*End to End; Color 1 (Purple) and Color 2 (Yellow)*/ },
                                        {}};

  frc2::InstantCommand SetBlinkinLeftBumper{[this] { blinkin.Set(.07); /*Fast Heartbeat; Color 1 (Purple)*/ },
                                        {}};

  frc2::InstantCommand SetBlinkinRightBumper{[this] { blinkin.Set(.27); /*Fast Heartbeat; Color 2 (Yellow)*/; },
                                        {}};

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

  frc2::Command* SetBlinkin(int inputMode);

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
};