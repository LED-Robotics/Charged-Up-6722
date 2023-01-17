// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include "ctre/Phoenix.h"

#include "Constants.h"

using namespace frc;

class TurretSubsystem : public frc2::SubsystemBase {
 public:
  TurretSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void ResetEncoder();

  void SetPower(double newPower);

  double GetPower();

  void SetTarget(double newTarget);

  int GetTarget();

  int GetPosition();

  void Off();

  void SetState(int newState);

  int GetState();
    
 private:
  int state = TurretConstants::kOff;
  int target = 0;
  double power = 0.0;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  WPI_VictorSPX motor;
  // WPI_TalonSRX motor;
  Encoder encoder;
  frc2::PIDController pidController;
};
