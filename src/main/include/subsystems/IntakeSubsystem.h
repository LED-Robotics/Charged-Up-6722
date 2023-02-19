// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <units/voltage.h>
#include <frc/controller/BangBangController.h>
#include "ctre/Phoenix.h"
#include <rev/CANSparkMax.h>

#include "Constants.h"

using namespace frc;

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Turns the intake state to kOff.
   */
  void Off();

  /**
   * Turns the intake state to kFullMode.
   */
  void On();

  /**
   * Turns the intake state to kPowerMode.
   * The intake uses the subsystems current
   * power setting while in this mode.
   */
  void UsePowerMode();

  /**
   * Sets the power to use while in kPowerMode.
   */
  void SetPower(double power);

  /**
   * Returns the current power setting of the intake.
   *
   * @return The current power setting of the intake
   */
  double GetPower();

  /**
   * Returns the current state of the intake.
   *
   * @return The current state of the intake
   */  
  int GetState();

  /**
   * Sets the position of the Intake wrist.
   */
  void SetPosition(double newPosition);

  /**
   * Returns the target position of the intake's Wrist.
   *
   * @return The target Intake wrist position
   */
  double GetTargetPosition();

  /**
   * Returns the current position of the intake's Wrist.
   *
   * @return The current Intake wrist position
   */
  double GetCurrentPosition();

  /**
   * Resets the Intake wrist encoder.
   */
  void ResetWristEncoder();

    
 private:
  int state = IntakeConstants::kOff;
  double power = 0.0;
  double position = 0.0;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  rev::CANSparkMax intakeMotor;
  WPI_TalonFX wristMotor;
};
