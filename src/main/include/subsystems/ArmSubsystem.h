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
  void SetTargetPosition(double newPosition);

  /**
   * Returns the current position of the left Arm's Falon500.
   */
  double GetLeftPosition();

  /**
   * Returns the current position of the right Arm's Falon500.
   */
  double GetRightPosition();
  
    
 private:
//  while the state is kOn the Arm will run at the current power setting
  int state = ArmConstants::kOff;
  double power = ArmConstants::kDefaultPower;
  double position = 0.0;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  // WPI_TalonSRX left;
  WPI_TalonFX left;
  // WPI_TalonSRX right;
  WPI_TalonFX right;
};