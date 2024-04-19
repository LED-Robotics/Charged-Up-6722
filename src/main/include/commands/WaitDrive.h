#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/DriveSubsystem.h"

/**
 * Timed drive Command that relies on accurate kinematics to ensure correct distance.
 */
class WaitDrive : public frc2::CommandHelper<frc2::Command, WaitDrive> {
 public:
  explicit WaitDrive(units::meter_t distance, units::velocity::meters_per_second_t speed, DriveSubsystem* driveRef);

  void Initialize() override;
  
  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
    DriveSubsystem *drive;
    units::meter_t target = 0.0_m;
    units::velocity::meters_per_second_t targetSpeed = 0.0_mps;
    units::second_t totalTime = 0.0_s;
    Timer doneTimer;
};