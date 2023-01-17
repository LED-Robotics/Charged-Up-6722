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

#include "Constants.h"

using namespace frc;

class FlywheelSubsystem : public frc2::SubsystemBase {
 public:
  FlywheelSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Sets the flywheel's state to kOff.
   */
  void Off();

  /**
   * Sets the RPM target to use while in kRpmMode.
   * @param rpm The RPM target to use.
   */
  void SetMotorRPM(double rpm);

  /**
   * Gets the distance of the left encoder.
   *
   * @return the flywheel's RPM
   */
  double GetRPM();

  /**
   * Set the power to use while in kPowerMode.
   */
  void SetMotorPower(double percent);

  /**
   * Returns the flywheel's power setting for kPowerMode.
   *
   * @return The power setting of the flywheel
   */
  double GetMotorPower();

  /**
   * Sets the current state of the flywheel.
   */
  void SetFlywheelState(int newState);

  /**
   * Returns the current state of the flywheel.
   *
   * @return The current state of the flywheel
   */
  int GetFlywheelState();
    
 private:
  int state = FlywheelConstants::kOff;
  double power = FlywheelConstants::kDefaultPower;
  double rpm = 0.0;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  WPI_TalonFX motor1;

  // frc::BangBangController rpmController{FlywheelConstants::bangBangThreshold};
};
