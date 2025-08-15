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

class WristSubsystem : public frc2::SubsystemBase {
 public:
  WristSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
            /* wrist FUNCTIONS */

  /**
   * Turns the Intake state to kAngleMode.
   */
  void WristOn();

  /**
   * Turns the Intake state to kOff.
   */
  void WristOff();

  /**
   * Sets the power for the Wrist to use when in kPowerMode.
   *
   * @param power the power for the wrist to use
   */
  void SetWristPower(double newPower);
  
  /**
   * Get the current power used by the Wrist.
   * 
   * @return current wrist power
   */
  double GetWristPower();

  /**
   * Sets the target angle of the Wrist.
   * 
   * @param newAngle new angle for the wrist
   */
  void SetTargetAngle(units::angle::degree_t newAngle);

  /**
   * Returns the current estimated angle of the wrist.
   * 
   * @return current wrist angle
   */
  units::angle::degree_t GetAngle();

  /**
   * Returns the position from the TalonFX motor controller.
   *
   * @return the TalonFX reported position
   */
  double GetWristPosition();

  /**
   * Returns whether the subsystem is at its intended target position.
   * 
   * @return If the wrist is at it's target
   */
  bool IsAtTarget();

  /**
   * Sets the current state of the Wrist.
   * 
   * @param newState the new state for the Wrist.
   */
  void SetWristState(int newState);
  
  /**
   * Returns the current state of the Wrist.
   *
   * @return The current state of the Wrist
   */
  int GetWristState();

  /**
   * Sets Wrist brake mode.
   * 
   * @param state turn the brakes on or off
   */
  void SetWristBrakeMode(bool state);

  /**
   * Initially configure onboard TalonFX settings for motors.
   */
  void ConfigWrist();

  /**
   * Create command to move Subsystem
   */
  frc2::CommandPtr GetMoveCommand(units::angle::degree_t target);
    
 private:
  // While the state is kOn the wrist will run on the angle mode.
  int state = WristConstants::WristStates::kWristAngleMode;
  double power = WristConstants::kWristDefaultPower;
  units::angle::degree_t angle{90_deg};
  units::angle::degree_t microAdjust{0_deg};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  hardware::TalonFX wrist;

  controls::PositionVoltage wristPosition{0_tr};
};
