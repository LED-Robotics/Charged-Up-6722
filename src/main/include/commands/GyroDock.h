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
class GyroDock : public frc2::CommandHelper<frc2::CommandBase, GyroDock> {
 public:
  explicit GyroDock(double targetSpeed, DriveSubsystem* driveRef);

  void Initialize() override;

  void Execute() override;

  bool IsFinished() override;

 private:
    double speed = 1.5;
    double initialPitch = 0.0;
    double changeThreshold = 3.0;
    double range = 15.0;
    const double kP = 0.35;
    bool initialApproachCompleted = false;
    bool sustained = false;
    bool tippedForward = false;
    bool backwards = false;
    Timer successTimer;
    units::second_t successTime = 2_s;
    DriveSubsystem *drive;
};