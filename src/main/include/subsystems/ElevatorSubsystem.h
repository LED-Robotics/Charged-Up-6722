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

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  /**
   * Turns the elevator state to kOff.
   */
  void Off();
  
  /**
   * Turns the elevator state to kPowerMode.
   */
  void On();

  /**
   * Sets the power for the elevator to use when in kPowerMode.
   *
   * @param power the power for the elevator to use
   */
  void SetPower(double newPower);

  /**
   * Returns the current state of the elevator.
   *
   * @return The current state of the elevator
   */
  int GetState();

  /**
   * Sets the current state of the elevator.
   */
  void SetState(int newState);

  /**
   * Returns the current position of the left elevator's Falon500.
   */
  void SetTargetPosition(double newPosition);

  /**
   * Returns the current position of the left elevator's Falon500.
   */
  double GetLeftPosition();

  /**
   * Returns the current position of the right elevator's Falon500.
   */
  double GetRightPosition();

  /**
   * Returns whether the subsystem is at its intended target position.
   */
  bool IsAtTarget();

  /**
   * Sets the brake mode of the elevator.
   *
   * @param state the state of the brakes
   */
  void SetBrakeMode(bool state);
  
    
 private:
//  while the state is kOn the elevator will run at the current power setting
  int state = ElevatorConstants::kPositionMode;
  double power = ElevatorConstants::kDefaultPower;
  double position = 0.0;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // limit switches at the bottom elevator position
  DigitalInput leftStopSensor;
  DigitalInput rightStopSensor;

  // The motor controllers
  // WPI_TalonSRX left;
  WPI_TalonFX left;
  // WPI_TalonSRX right;
  WPI_TalonFX right;
};
