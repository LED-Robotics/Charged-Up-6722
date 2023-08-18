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
#include <frc/DigitalInput.h>
#include "ctre/Phoenix.h"

#include "Constants.h"

using namespace frc;

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  /**
   * Turns the Arm state to kOff.
   */
  void Off();
  
  /**
   * Turns the Arm state to kPowerMode.
   */
  void On();

  /**
   * Sets the power for the Arm to use when in kPowerMode.
   *
   * @param power the power for the Arm to use
   */
  void SetPower(double newPower);

  /**
   * Returns the current state of the Arm.
   *
   * @return The current state of the Arm
   */
  int GetState();

  /**
   * Sets the current state of the Arm.
   */
  void SetState(int newState);
  
  /**
   * Returns the current position of the left Arm's Falon500.
   */
  double GetLeftPosition();

  /**
   * Returns the current position of the right Arm's Falon500.
   */
  double GetRightPosition();

  /**
   * Returns the current estimated angle of the arm Subsystem.
   */
  double GetAngle();

  /**
   * Sets the target angle of the Arm.
   */
  void SetTargetAngle(double newAngle);

  /**
   * Returns whether the subsystem is at its intended target position.
   */
  bool IsAtTarget();

  /**
   * Sets the state of the Arm brakes.
   *
   * @param state the state of the brakes.
   */
  void SetBrakeMode(bool state);

  /**
   * Initially configure onboard TalonFX settings for motors.
   */
  void ConfigMotors();

  /**
   * Set arm to cone placement mode.
   */
  void SetConeMode();

  /**
   * Set arm to cube placement mode.
   */
  void SetCubeMode();
  /**
   * Returns whether the robot is in cone placement mode.
   *
   * @return cone mode state
   */
  bool GetConeMode();
    
 private:
//  while the state is kOn the Arm will run at the current power setting
  int state = ArmConstants::kPositionMode;
  double power = ArmConstants::kDefaultPower;
  double angle = 11.5;

  bool isCone = true; // flag for cone/cube mode

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  // WPI_TalonSRX left;
  WPI_TalonFX left;
  // WPI_TalonSRX right;
  WPI_TalonFX right;
};
