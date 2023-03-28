#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/IntakeSubsystem.h"

/**
 * A command that sets the position of the elevator, intake, and arm subsystems. 
 * This command controls the order in which the subsystems move based on the target position,
 * as opposed to the original method of moving all three simultaneously all the time.
 */
class SetIntakePower : public frc2::CommandHelper<frc2::CommandBase, SetIntakePower> {
 public:
  explicit SetIntakePower(double newPower, IntakeSubsystem* intakeRef);

  void Initialize() override;

  bool IsFinished() override;

 private:
    IntakeSubsystem *intake;
    double power = 0.0;
};