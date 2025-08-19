// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include "Constants.h"

using namespace frc;
using namespace ctre::phoenix6;

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
            /* arm FUNCTIONS */

  /**
   * Turns the Intake state to kAngleMode.
   */
  void ArmOn();

  /**
   * Turns the Intake state to kOff.
   */
  void ArmOff();

  /**
   * Sets the power for the Arm to use when in kPowerMode.
   *
   * @param power the power for the arm to use
   */
  void SetArmPower(double newPower);
  
  /**
   * Get the current power used by the Arm.
   * 
   * @return current arm power
   */
  double GetArmPower();

  /**
   * Sets the target angle of the Arm.
   * 
   * @param newAngle new angle for the arm
   */
  void SetTargetAngle(units::angle::degree_t newAngle);

  /**
   * Returns the current estimated angle of the arm.
   * 
   * @return current arm angle
   */
  units::angle::degree_t GetAngle();

  /**
   * Returns the position from the TalonFX motor controller.
   *
   * @return the TalonFX reported position
   */
  double GetArmPosition();

  /**
   * Returns whether the subsystem is at its intended target position.
   * 
   * @return If the arm is at it's target
   */
  bool IsAtTarget();

  /**
   * Sets the current state of the Arm.
   * 
   * @param newState the new state for the Arm.
   */
  void SetArmState(int newState);
  
  /**
   * Returns the current state of the Arm.
   *
   * @return The current state of the Arm
   */
  int GetArmState();

  /**
   * Sets Arm brake mode.
   * 
   * @param state turn the brakes on or off
   */
  void SetArmBrakeMode(bool state);

  /**
   * Initially configure onboard TalonFX settings for motors.
   */
  void ConfigArm();

  /**
   * Create command to move Subsystem
   */
  frc2::CommandPtr GetMoveCommand(units::angle::degree_t target);
    
 private:
  // While the state is kOn the arm will run on the angle mode.
  int state = ArmConstants::ArmStates::kArmAngleMode;
  double power = ArmConstants::kArmDefaultPower;
  units::angle::degree_t angle{-90_deg};
  units::angle::degree_t microAdjust{0_deg};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  hardware::TalonFX leftArm;
  hardware::TalonFX rightArm;

  controls::PositionVoltage armPosition{0_tr};
};
