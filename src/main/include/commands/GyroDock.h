#pragma once

#include "Constants.h"
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

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
    DriveSubsystem* drive;
};