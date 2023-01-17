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
   * Turns the elevator state to kOn.
   */
  void On();

  /**
   * Sets the elevator to run at the given power.
   *
   * @param power the power to run the elevator at
   */
  void SetPower(double newPower);

  /**
   * Returns the current state of the elevator.
   *
   * @return The current state of the elevator
   */
  int GetState();
    
 private:
//  while the state is kOn the elevator will run at the current power setting
  int state = ElevatorConstants::kOff;
  double power = ElevatorConstants::kDefaultPower;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  // WPI_TalonSRX bottom;
  WPI_VictorSPX bottom;
  // WPI_TalonSRX top;
  WPI_VictorSPX top;
};
