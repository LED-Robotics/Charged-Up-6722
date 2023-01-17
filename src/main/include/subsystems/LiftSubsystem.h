// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/Phoenix.h"

#include "Constants.h"

using namespace frc;

class LiftSubsystem : public frc2::SubsystemBase {
 public:
  LiftSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  /**
   * Turns the lift state to kOff.
   */
  void Off();
  
  /**
   * Set the lift state to the passed argument.
   * @param state the target state for the lift
   */
  void SetState(int newState);

  /**
   * Returns the current state of the lift.
   *
   * @return The current state of the lift
   */
  int GetState();

  /**
   * Sets the power of the lift.
   *
   * @param newPower The power to set the lift to
   */
  void SetPower(double newPower);
    
 private:
//  while the state is kOn the lift will run at the current power setting
  int state = LiftConstants::kOff;
  double power = 0.0;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  WPI_TalonSRX motor;
  // WPI_VictorSPX motor;
};
