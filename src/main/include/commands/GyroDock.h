#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/DriveSubsystem.h"

/**
 * Command to dock and engage on the charge station.
 */
class GyroDock : public frc2::CommandHelper<frc2::Command, GyroDock> {
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