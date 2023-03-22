#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/DriveSubsystem.h"

/**
 * A command that sets the position of the elevator, intake, and arm subsystems. 
 * This command controls the order in which the subsystems move based on the target position,
 * as opposed to the original method of moving all three simultaneously all the time.
 */
class WaitDrive : public frc2::CommandHelper<frc2::CommandBase, WaitDrive> {
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