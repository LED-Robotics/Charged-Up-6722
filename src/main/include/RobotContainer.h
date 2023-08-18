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
#include "commands/SetIntakePower.h"
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
   */  
  void SetSlew(bool state);

 private:

  // The driver's controller
  frc2::CommandXboxController controller{OIConstants::kDriverControllerPort};
  // The partner controller
  frc2::CommandXboxController controller2{OIConstants::kCoDriverControllerPort};

  //Blinkin LED controller
  Spark blinkin{BlinkinConstants::kBlinkinPort};
  

  // The robot's subsystems
  DriveSubsystem m_drive;
  
  ElevatorSubsystem elevator;
  
  ArmSubsystem arm;

  // needs a ref to the arm so it can determine its angle relative to the floor
  IntakeSubsystem intake{&arm};

  LimelightSubsystem armLimelight{"limelight-arm"};

  // robot offset relative to the field as it started
  // used for AprilTag odom updates
  units::degree_t startOffset{180.0};

  // variable to track the target station
  int targetStation = 4;

  // flag used to trigger auto-docking
  bool dockActive = false;

  // flag used to trigger auto-dock maintain
  bool maintainActive = false;
  
  // flag used to trigger auto-aligning
  bool positionHoldActive = false;
  
  // flag to maintain intake at holding power
  bool intakeHold = false;

  // flag to drive using field-centric positions
  bool fieldCentric = true;

  // flag to disable AprilTag odom updates
  bool tagOverrideDisable = false;

  // flag to update odom if a valid AprilTag is seen
  bool validTag = false;

  // Auton subroutine Triggers

  // Trigger to cancel all auton subroutines. Triggers on any joystick movement above a deadzone while a Command is active.
  frc2::Trigger moveRoutineCancel{[this]() { return (positionHoldActive || dockActive || maintainActive) && 
  (abs(controller.GetLeftX()) > 0.1 || abs(controller.GetLeftY()) > 0.1 || abs(controller.GetRightX()) > 0.1); }};
  
  // Trigger auto-align on flag
  frc2::Trigger holdingTrigger{[this]() { return positionHoldActive; }};

  // Trigger auto-dock on flag
  frc2::Trigger dockTrigger{[this]() { return dockActive; }};
  
  // Trigger auto-dock maintain on flag
  frc2::Trigger maintainTrigger{[this]() { return maintainActive; }};
    
  // Trigger odom update on flag
  frc2::Trigger odomTrigger{[this]() { return validTag; }};


  // Test to make sure actual turn speed matches target
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

  // Test to make sure actual translation speed matches target
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

  // Triggers for main and partner D-PAD positions

  frc2::Trigger mainDpadUp{[this]() { return controller.GetPOV() == 0; }};
  frc2::Trigger mainDpadRight{[this]() { return controller.GetPOV() == 90; }};
  frc2::Trigger mainDpadDown{[this]() { return controller.GetPOV() == 180; }};
  frc2::Trigger mainDpadLeft{[this]() { return controller.GetPOV() == 270; }};

  frc2::Trigger partnerDpadUp{[this]() { return controller2.GetPOV() == 0; }};
  frc2::Trigger partnerDpadRight{[this]() { return controller2.GetPOV() == 90; }};
  frc2::Trigger partnerDpadDown{[this]() { return controller2.GetPOV() == 180; }};
  frc2::Trigger partnerDpadLeft{[this]() { return controller2.GetPOV() == 270; }};

  // Command to control elev, arm, intake manually within preset position constraints.
  // The AWARD-WINNING GIGAMUNGUS MODE!
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

  // Gutted auto-dock to maintain robot position on top.
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

  
  // update odom based on Limelight's AprilTag megabotpose estimation
  frc2::InstantCommand updateOdometry{
        [this]() { 
          if(!tagOverrideDisable) {
            std::vector<double> pose = armLimelight.GetBotPos();
            if(pose[0] == 0.0 && pose[1] == 0.0 && pose[5] == 0.0) return;  
            m_drive.ResetOdometry({units::meter_t{pose[0]}, units::meter_t{pose[1]}, {m_drive.GetRotation().Degrees() + startOffset}});
          }
        }, {}};

  // Command to repetitively call odom update
  frc2::RepeatCommand repeatOdom{std::move(updateOdometry)};

  // Toggles for auton subroutines
  frc2::InstantCommand togglePositionHold{[this] { positionHoldActive = !positionHoldActive; },
  {}};

  frc2::InstantCommand toggleDocking{[this] { dockActive = !dockActive; },
  {}};

  frc2::InstantCommand toggleMaintain{[this] { maintainActive = !maintainActive; },
  {}};

  // Increment target station
  frc2::InstantCommand incrementStation{[this] { 
    targetStation += targetStation < 8 ? 1 : 0; 
    m_drive.SetLimiting(false);
    m_drive.SetPoseToHold(GetTargetPose());
    m_drive.StartHolding();
    UpdateStationUI();
    },
  {}};

  // Decrement target station
  frc2::InstantCommand decrementStation{[this] { 
    targetStation -= targetStation > 0 ? 1 : 0; 
    m_drive.SetLimiting(false);
    m_drive.SetPoseToHold(GetTargetPose());
    m_drive.StartHolding();
    UpdateStationUI();
    },
  {}};

  // Set target station to single substation
  frc2::InstantCommand setToSubstation{[this] { 
    targetStation = 9;
    m_drive.SetLimiting(false);
    m_drive.SetPoseToHold(GetTargetPose());
    m_drive.StartHolding();
    UpdateStationUI();
    },
  {}};

  // funny rumble Commands
  frc2::InstantCommand rumblePrimaryOn{[this] { controller.SetRumble(GenericHID::kBothRumble, 1.0); },
                                        {}};

  frc2::InstantCommand rumbleSecondaryOn{[this] { controller2.SetRumble(GenericHID::kBothRumble, 1.0); },
                                        {}};
  
  frc2::InstantCommand rumblePrimaryOff{[this] { controller.SetRumble(GenericHID::kBothRumble, 0.0); },
                                        {}};

  frc2::InstantCommand rumbleSecondaryOff{[this] { controller2.SetRumble(GenericHID::kBothRumble, 0.0); },
                                        {}};
  
  // LED control Commands
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
  /**
   * Find whether the robot is on the blue or red alliance as set by the FMS/DriverStation.
   *
   * @return A bool for if the robot is on the blue alliance
   */
  bool IsBlue();
  void UpdateStationUI();
  /**
   * Iterate through and calculate the closest station position. Return -1 if too far.
   *
   * @return The closest valid station
   */
  int GetClosestPosition();
  /**
   * Generate a SetPosition Command from a position.
   *
   * @return A pointer to an appropriate SetPosition Command
   */
  frc2::Command* GetPositionCommand(int position);
  /**
   * Return one of two Commands based on whether a partner controller is connected.
   *
   * @return The appropriate Command* based on partner controller status
   */
  frc2::Command* HandlePartnerCommands(frc2::Command* solo, frc2::Command* partner);
  /**
   * Return a pointer to an empty Command that will do nothing when run.
   *
   * @return A Command* to an empty Command
   */
  frc2::Command* GetEmptyCommand();
  /**
   * Find currently selected station and make a Pose2d its position.
   *
   * @return A Pose2d to the currently selected station
   */
  frc::Pose2d GetTargetPose();

  // config DriveSubsystem to hold at the target station position
  frc2::CommandPtr startHolding{frc2::cmd::RunOnce([this] { 
    targetStation = GetClosestPosition();
    if(targetStation == -1) positionHoldActive = false;
    m_drive.SetLimiting(false);
    m_drive.SetPoseToHold(GetTargetPose());
    m_drive.StartHolding();
    UpdateStationUI();
  }, {&m_drive})};

  // should be run continuously to hold robot position at station
  frc2::CommandPtr holdPosition{frc2::cmd::Run([this] { 
      auto speeds = m_drive.CalculateHolding();
      m_drive.Drive(speeds.vx, speeds.vy, {speeds.omega}, true);
    }, {&m_drive})};

  // reenable SlewRateLimiters
  std::function<void(bool)> endHolding {[this](bool interrupted) { 
    m_drive.SetLimiting(true);
  }};

  // reenable SlewRateLimiters
  std::function<void(bool)> endDocking {[this](bool interrupted) { 
    m_drive.SetLimiting(true);
  }};

  // PUNCH!
  frc2::CommandPtr punchObject{frc2::SequentialCommandGroup(
    // spit out element
    SetIntakePower(-1.0, &intake),
    // wait for element to fly out
    frc2::WaitCommand(0.15_s),
    // PUNCH!
    SetPosition(2, &elevator, &arm, &intake),
    // turn off intake
    SetIntakePower(0.0, &intake),
    // return to stored position
    SetPosition(0, &elevator, &arm, &intake)).ToPtr()
  };

  // sets each auton subroutine flag to false to cancel them
  frc2::CommandPtr cancelMoves{frc2::cmd::RunOnce([this] { 
    positionHoldActive = false;
    dockActive = false;
    maintainActive = false;
  }, {})};

  // Positional arrays for station targets. Used for auto-alignment
  const frc::Translation2d BlueHoldPositions[10]{{-6.14_m, -3.47_m}, {-6.14_m, -2.84_m}, {-6.14_m, -2.23_m}, {-6.14_m, -1.65_m}, {-6.14_m, -1.15_m}, {-6.14_m, -0.49_m}, {-6.14_m, -0.12_m}, {-6.14_m, 0.4_m}, {14.25_m, 1.0_m}, {14.25_m, 7.5_m}};
  const frc::Translation2d RedHoldPositions[10]{{5.91_m, -3.45_m}, {5.91_m, -3.32_m}, {5.91_m, -2.38_m}, {5.91_m, -1.63_m}, {5.91_m, -1.29_m}, {5.91_m, -1.44_m}, {5.91_m, -1.74_m}, {5.91_m, -2.04_m}, {5.91_m, -2.34_m}, {2.3_m, 7.5_m}};
  
  // Debug Command for old auto-angle alignment.
  TurnTo turnTo90{3.0, &m_drive};

  // Debug Command for auto-position align.
  frc2::CommandPtr toFive{ToPoint({5.0_m, 5.0_m, {90_deg}}, &m_drive)};
  
  // Command for docking in teleop.
  frc2::CommandPtr teleDock{GyroDock(-1.8, &m_drive)};
  
  // Auton routines timeout after 15.0 seconds as a precaution. If running them several times, RESTART THE ROBOT CODE. The timer does not reset after the command finishes. 
  
  // Simple dock and engage auton.
  frc2::CommandPtr dock{GyroDock(1.5, &m_drive).WithTimeout(15.0_s)};

  // Two point cube auton for blue alliance.
  frc2::CommandPtr wallNoBalance{WallNoBalance(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};

  // Two point cube auton swapped for red alliance.
  frc2::CommandPtr wallNoBalanceRed{WallNoBalanceRed(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};
  
  // Auton which places an element low and gets the mobility bonus.
  frc2::CommandPtr lowPlaceThenBreak{LowPlaceThenBreak(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};
  
  // Auton which places an element high and gets the mobility bonus.
  frc2::CommandPtr placeThenBreak{PlaceThenBreak(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};

  // Auton which places an element high, gets the mobility bonus, docks and then engages.
  frc2::CommandPtr highDock{HighDock(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};

  // Auton which places an element low, gets the mobility bonus, docks and then engages.
  frc2::CommandPtr lowDock{LowDock(&m_drive, &elevator, &arm, &intake).WithTimeout(15.0_s)};

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> chooser;
};