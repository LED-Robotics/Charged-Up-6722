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
#include <iostream>
#include "ArmSubsystem.h"

#include "Constants.h"

using namespace frc;

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem(ArmSubsystem *reference);

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
  void SetPower(double newPower);
  void SetWristPower(double newPower);

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
   * Sets the current state of the intake.
   */  
  void SetState(int newState);

  /**
   * Returns the current state of the intake's wrist.
   *
   * @return The current state of the intake's wrist
   */  
  int GetWristState();

  /**
   * Sets the current state of the intake's wrist.
   */  
  void SetWristState(int newState);

  /**
   * Sets the position of the Intake wrist.
   */
  void SetTargetPosition(double newPosition);

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
   * Sets the target angle of the Intake wrist.
   */
  void SetTargetAngle(double newAngle);

  /**
   * Returns the target position of the intake's Wrist.
   *
   * @return The target Intake wrist position
   */
  double GetTargetAngle();

  /**
   * Returns the current angle of the intake's Wrist.
   *
   * @return The current Intake wrist angle
   */
  double GetCurrentAngle();

  /**
   * Resets the Intake wrist encoder.
   */
  void ResetWristEncoder();

  /**
   * Returns whether the subsystem is at its intended target position.
   */
  bool IsAtTarget();

  /**
   * Sets Intake brake mode.
   */
  void SetBrakeMode(bool state);

  /**
   * Initially configure onboard TalonFX settings for motors.
   */
   void ConfigMotors();

    
 private:
  ArmSubsystem *arm;
  int state = IntakeConstants::kOff;
  int wristState = IntakeConstants::kAngleMode;
  double power = 0.0;
  double wristPower = 0.0;
  double position = 4000;
  double angle = 0.0;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  WPI_TalonFX intakeMotor;
  WPI_TalonFX wristMotor;
};
