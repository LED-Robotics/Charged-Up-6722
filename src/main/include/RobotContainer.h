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
#include "units/angle.h"
#include <ctre\Phoenix.h>

#include "Constants.h"
#include "commands/ToPoint.h"
#include "commands/TurnTo.h"
#include "commands/SetPosition.h"
#include "commands/WallNoBalance.h"
#include "commands/WallNoBalanceRed.h"
#include "commands/LowPlaceThenBreak.h"
#include "commands/PlaceThenBreak.h"
#include "commands/HighDock.h"
#include "commands/LowDock.h"
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

  void EnableTagTracking();

  void DisableTagTracking();

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

  LimelightSubsystem armLimelight{"limelight-arm"};

  units::degree_t startOffset{180.0};

  int targetStation = 4;

  bool dockActive = false;

  bool maintainActive = false;
  
  bool positionHoldActive = false;
  
  bool intakeHold = false;

  bool fieldCentric = true;

  bool tagOverrideDisable = false;

  bool validTag = false;

  frc2::Trigger moveRoutineCancel{[this]() { return (positionHoldActive || dockActive || maintainActive) && 
  (abs(controller.GetLeftX()) > 0.1 || abs(controller.GetLeftY()) > 0.1 || abs(controller.GetRightX()) > 0.1); }};
  
  frc2::Trigger holdingTrigger{[this]() { return positionHoldActive; }};

  frc2::Trigger dockTrigger{[this]() { return dockActive; }};
  
  frc2::Trigger maintainTrigger{[this]() { return maintainActive; }};
    
  frc2::Trigger odomTrigger{[this]() { return validTag; }};

  // frc::Field2d field;

  frc2::SequentialCommandGroup turnRateTest{    
      frc2::InstantCommand(
      [this]() { 
        m_drive.Drive(0.0_mps, 0_mps, 90_deg_per_s, false);
      }, {&m_drive}),
      frc2::WaitCommand(1.0_s),
      frc2::InstantCommand(
      [this]() { 
        m_drive.Drive(0.0_mps, 0_mps, 0_deg_per_s, false);
      }, {&m_drive}),
   };

   frc2::SequentialCommandGroup driveRateTest{    
      frc2::InstantCommand(
      [this]() { 
        m_drive.Drive(0.0_mps, 1.0_mps, 0_deg_per_s, false);
      }, {&m_drive}),
      frc2::WaitCommand(1.0_s),
      frc2::InstantCommand(
      [this]() { 
        m_drive.Drive(0.0_mps, 0_mps, 0_deg_per_s, false);
      }, {&m_drive}),
   };

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

  frc2::CommandPtr maintainEngage{frc2::cmd::Run(
      [this] {
        m_drive.SetLimiting(false);
        double current = m_drive.GetPitch();
        double range = 20.0;
        double kP = 0.35;
        double error = (0.0 - current) / range * kP;
        double changeThreshold = 3.0;
        if(current > 0.0 + (changeThreshold / 2) && current < 0.0 - (changeThreshold / 2)) {
          m_drive.Drive(0_mps, 0_mps, 0_deg_per_s, false);
        } else {
          m_drive.Drive(units::meters_per_second_t{(-1.8 * 0.43) * error}, 0_mps, 0_deg_per_s, false);
        }
      },
      {&m_drive})};

  

  frc2::InstantCommand updateOdometry{
        [this]() { 
          if(!tagOverrideDisable) {
            std::vector<double> pose = armLimelight.GetBotPos();
            if(pose[0] == 0.0 && pose[1] == 0.0 && pose[5] == 0.0) return;  
            m_drive.ResetOdometry({units::meter_t{pose[0]}, units::meter_t{pose[1]}, {m_drive.GetRotation().Degrees() + startOffset}});
          }
        }, {}};

  frc2::RepeatCommand repeatOdom{std::move(updateOdometry)};


  frc2::InstantCommand toggleFieldCentric{[this] { fieldCentric = !fieldCentric; },
                                        {}};

  frc2::InstantCommand togglePositionHold{[this] { positionHoldActive = !positionHoldActive; },
  {}};

  frc2::InstantCommand toggleDocking{[this] { dockActive = !dockActive; },
  {}};

  frc2::InstantCommand toggleMaintain{[this] { maintainActive = !maintainActive; },
  {}};

  frc2::InstantCommand incrementStation{[this] { 
    targetStation += targetStation < 8 ? 1 : 0; 
    m_drive.SetLimiting(false);
    m_drive.SetPoseToHold(GetTargetPose());
    m_drive.StartHolding();
    UpdateStationUI();
    },
  {}};

  frc2::InstantCommand decrementStation{[this] { 
    targetStation -= targetStation > 0 ? 1 : 0; 
    m_drive.SetLimiting(false);
    m_drive.SetPoseToHold(GetTargetPose());
    m_drive.StartHolding();
    UpdateStationUI();
    },
  {}};

  frc2::InstantCommand setToSubstation{[this] { 
    targetStation = 9;
    m_drive.SetLimiting(false);
    m_drive.SetPoseToHold(GetTargetPose());
    m_drive.StartHolding();
    UpdateStationUI();
    },
  {}};

  frc2::InstantCommand rumblePrimaryOn{[this] { controller.SetRumble(GenericHID::kBothRumble, 1.0); },
                                        {}};

  frc2::InstantCommand rumbleSecondaryOn{[this] { controller2.SetRumble(GenericHID::kBothRumble, 1.0); },
                                        {}};
  
  frc2::InstantCommand rumblePrimaryOff{[this] { controller.SetRumble(GenericHID::kBothRumble, 0.0); },
                                        {}};

  frc2::InstantCommand rumbleSecondaryOff{[this] { controller2.SetRumble(GenericHID::kBothRumble, 0.0); },
                                        {}};
  
  
  frc2::InstantCommand SetBlinkinAButton{[this] { 
    arm.SetConeMode();
    blinkin.Set(.41); /*End to End; Color 1 (Purple) and Color 2 (Yellow)*/ },
                                        {&arm}};

  frc2::InstantCommand SetBlinkinLeftBumper{[this] { 
    arm.SetCubeMode();
    blinkin.Set(.07); /*Fast Heartbeat; Color 1 (Purple)*/ },
                                        {&arm}};

  frc2::InstantCommand SetBlinkinRightBumper{[this] { 
    arm.SetConeMode();
    blinkin.Set(.27); /*Fast Heartbeat; Color 2 (Yellow)*/; },
                                        {&arm}};

  bool IsBlue();

  void UpdateStationUI();

  int GetClosestPosition();

  frc2::Command* GetPositionCommand(int position);

  frc2::Command* HandlePartnerCommands(frc2::Command* solo, frc2::Command* partner);

  frc2::Command* GetEmptyCommand();

  frc2::Command* SetBlinkin(int inputMode);

  frc::Pose2d GetTargetPose();

  frc2::CommandPtr startHolding{frc2::cmd::RunOnce([this] { 
    targetStation = GetClosestPosition();
    if(targetStation == -1) positionHoldActive = false;
    m_drive.SetLimiting(false);
    m_drive.SetPoseToHold(GetTargetPose());
    m_drive.StartHolding();
    UpdateStationUI();
  }, {&m_drive})};

  frc2::CommandPtr holdPosition{frc2::cmd::Run([this] { 
      auto speeds = m_drive.CalculateHolding();
      m_drive.Drive(speeds.vx, speeds.vy, {speeds.omega}, true);
    }, {&m_drive})};

  // frc2::CommandPtr endHolding{frc2::InstantCommand().ToPtr()};
  std::function<void(bool)> endHolding {[this](bool interrupted) { 
    m_drive.SetLimiting(true);
  }};

  std::function<void(bool)> endDocking {[this](bool interrupted) { 
    m_drive.SetLimiting(true);
  }};

  frc2::CommandPtr punchObject{frc2::SequentialCommandGroup(
    frc2::InstantCommand([this] { 
      intake.SetPower(-1.0);
    }, {&intake}),
    frc2::WaitCommand(0.15_s),
    SetPosition(2, &elevator, &arm, &intake),
    frc2::InstantCommand([this] { 
      intake.SetPower(0.0);
    }, {&intake}),
    SetPosition(0, &elevator, &arm, &intake)).ToPtr()
  };

  frc2::CommandPtr cancelMoves{frc2::cmd::RunOnce([this] { 
    positionHoldActive = false;
    dockActive = false;
    maintainActive = false;
  }, {})};

  // const frc::Translation2d OldHoldPositions[10]{{2.10_m, 5.0_m}, {2.10_m, 4.42_m}, {2.10_m, 3.88_m}, {2.10_m, 3.30_m}, {2.10_m, 2.75_m}, {2.10_m, 2.20_m}, {2.10_m, 1.60_m}, {2.10_m, 1.05_m}, {2.10_m, 0.5_m}, {14.25_m, 7.5_m}};
  
  const frc::Translation2d BlueHoldPositions[10]{{-6.14_m, -3.47_m}, {-6.14_m, -2.84_m}, {-6.14_m, -2.23_m}, {-6.14_m, -1.65_m}, {-6.14_m, -1.15_m}, {-6.14_m, -0.49_m}, {-6.14_m, -0.12_m}, {-6.14_m, 0.4_m}, {14.25_m, 1.0_m}, {14.25_m, 7.5_m}};
  const frc::Translation2d RedHoldPositions[10]{{5.91_m, -3.45_m}, {5.91_m, -3.32_m}, {5.91_m, -2.38_m}, {5.91_m, -1.63_m}, {5.91_m, -1.29_m}, {5.91_m, -1.44_m}, {5.91_m, -1.74_m}, {5.91_m, -2.04_m}, {5.91_m, -2.34_m}, {2.3_m, 7.5_m}};
  
  TurnTo turnTo90{3.0, &m_drive};

  frc2::CommandPtr toFive{ToPoint({5.0_m, 5.0_m, {90_deg}}, &m_drive).WithTimeout(15.0_s)};

  frc2::CommandPtr teleDock{GyroDock(-1.8, &m_drive)};
  
  frc2::CommandPtr dock{GyroDock(1.5, &m_drive).WithTimeout(15.0_s)};

  frc2::CommandPtr wallNoBalance{WallNoBalance(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};

  frc2::CommandPtr wallNoBalanceRed{WallNoBalanceRed(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};
  
  frc2::CommandPtr lowPlaceThenBreak{LowPlaceThenBreak(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};
  
  frc2::CommandPtr placeThenBreak{PlaceThenBreak(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};

  frc2::CommandPtr highDock{HighDock(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};

  frc2::CommandPtr lowDock{LowDock(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> chooser;
};