#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include <frc/controller/PIDController.h>

#include "subsystems/DriveSubsystem.h"

/**
 * A command that sets the position of the elevator, intake, and arm subsystems. 
 * This command controls the order in which the subsystems move based on the target position,
 * as opposed to the original method of moving all three simultaneously all the time.
 */
class TurnTo : public frc2::CommandHelper<frc2::CommandBase, TurnTo> {
 public:
  explicit TurnTo(double target, DriveSubsystem* driveRef);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
    double angle = 0.0;
    bool flipped = false;
    const double kP = 1.4;
    frc2::PIDController pidController{kP, 0.0, 0.0};
    DriveSubsystem *drive;
};