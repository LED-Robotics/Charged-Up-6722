// Copyright (c) FIRST and other WPILib contributors.
// Open Soruce Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in root directory of this project.

#include "subsystems/FalconVelocityController.h"

#include <numbers>

FalconVelocityController::FalconVelocityController(int port, std::string const &canbus = "") : WPI_TalonFX(port, canbus) {

}