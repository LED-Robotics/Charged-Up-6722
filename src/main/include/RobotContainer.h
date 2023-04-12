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
#include "commands/TurnTo.h"
#include "commands/TrajectoryRelative.h"
#include "commands/TrajectoryAbsolute.h"
#include "commands/PathPlannerFollow.h"
#include "commands/SetPosition.h"
#include "commands/WallNoBalance.h"
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
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>

using namespace pathplanner;

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

  LimelightSubsystem armLimelight{"limelight-arm"};

  units::degree_t startOffset{180.0};

  int targetStation = 4;

  bool positionHoldActive = false;
  
  bool intakeHold = false;

  bool fieldCentric = true;

  bool validTag = false;

  frc2::Trigger moveRoutineCancel{[this]() { return positionHoldActive && 
  (abs(controller.GetLeftX()) > 0.1 || abs(controller.GetLeftY()) > 0.1 || abs(controller.GetRightX()) > 0.1); }};
  
  frc2::Trigger holdingTrigger{[this]() { return positionHoldActive; }};
    
  frc2::Trigger odomTrigger{[this]() { return validTag; }};

  frc::Field2d field;

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;
  // eventMap.emplace("marker1", std::make_shared<frc2::PrintCommand>("Passed Marker 1"));
  // eventMap.emplace("intakeDown", std::make_shared<frc2::PrintCommand>("Passed Marker 2"));

  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this could be in RobotContainer along with your subsystems

  SwerveAutoBuilder autoBuilder{
    [this]() { return m_drive.GetPose(); }, // Function to supply current robot pose
    [this](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
    PIDConstants(0.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    PIDConstants(0.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    [this](auto speeds) { m_drive.Drive(speeds.vx, speeds.vy, {speeds.omega}, false); }, // Output function that accepts field relative ChassisSpeeds
    eventMap, // Our event map
    { &m_drive }, // Drive requirements, usually just a single drive subsystem
    true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  };

  frc2::CommandPtr testAuto{autoBuilder.fullAuto(PathPlanner::loadPath("AutoPath", {PathConstraints(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration)}))};
  frc2::CommandPtr testPath{autoBuilder.fullAuto(PathPlanner::loadPath("TestPath", {PathConstraints(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration)}))};
  frc2::CommandPtr testRotate{autoBuilder.fullAuto(PathPlanner::loadPath("TestRotate", {PathConstraints(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration)}))};

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

  

  frc2::InstantCommand updateOdometry{
        [this]() { 
          std::vector<double> pose = armLimelight.GetBotPos();
          if(pose[0] == 0.0 && pose[1] == 0.0 && pose[5] == 0.0) return;  
          m_drive.ResetOdometry({units::meter_t{pose[0]}, units::meter_t{pose[1]}, {m_drive.GetRotation().Degrees() + startOffset}});
        }, {}};

  frc2::RepeatCommand repeatOdom{std::move(updateOdometry)};


  frc2::InstantCommand toggleFieldCentric{[this] { fieldCentric = !fieldCentric; },
                                        {}};

  frc2::InstantCommand togglePositionHold{[this] { positionHoldActive = !positionHoldActive; },
  {}};

  frc2::InstantCommand incrementStation{[this] { 
    targetStation += targetStation < 8 ? 1 : 0; 
    m_drive.SetLimiting(false);
    m_drive.SetPoseToHold(GetTargetPose());
    m_drive.StartHolding();
    },
  {}};

  frc2::InstantCommand decrementStation{[this] { 
    targetStation -= targetStation > 0 ? 1 : 0; 
    m_drive.SetLimiting(false);
    m_drive.SetPoseToHold(GetTargetPose());
    m_drive.StartHolding();
    },
  {}};

  frc2::InstantCommand setToSubstation{[this] { 
    targetStation = 9;
    m_drive.SetLimiting(false);
    m_drive.SetPoseToHold(GetTargetPose());
    m_drive.StartHolding();
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

  int GetClosestPosition();

  frc2::Command* GetPositionCommand(int position);

  frc2::Command* GetRelativePathCommand(const Pose2d& start, const std::vector<Translation2d>& interiorWaypoints,
    const Pose2d& end, const TrajectoryConfig& config);

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
  }, {&m_drive})};

  frc2::CommandPtr holdPosition{frc2::cmd::Run([this] { 
      auto speeds = m_drive.CalculateHolding();
      m_drive.Drive(speeds.vx, speeds.vy, {speeds.omega}, true);
    }, {&m_drive})};

  // frc2::CommandPtr endHolding{frc2::InstantCommand().ToPtr()};
  std::function<void(bool)> endHolding {[this](bool interrupted) { 
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
  }, {})};

  const frc::Translation2d HoldPositions[10]{{2.10_m, 5.0_m}, {2.10_m, 4.42_m}, {2.10_m, 3.88_m}, {2.10_m, 3.30_m}, {2.10_m, 2.75_m}, {2.10_m, 2.20_m}, {2.10_m, 1.60_m}, {2.10_m, 1.05_m}, {2.10_m, 0.5_m}, {14.25_m, 7.5_m}};
  
  TurnTo turnTo90{3.0, &m_drive};

  frc2::CommandPtr dock{GyroDock(1.5, &m_drive).WithTimeout(15.0_s)};

  frc2::CommandPtr wallNoBalance{WallNoBalance(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};
  
  frc2::CommandPtr lowPlaceThenBreak{LowPlaceThenBreak(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};
  
  frc2::CommandPtr placeThenBreak{PlaceThenBreak(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};

  frc2::CommandPtr highDock{HighDock(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};

  frc2::CommandPtr lowDock{LowDock(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> chooser;
};