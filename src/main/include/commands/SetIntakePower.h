#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/IntakeSubsystem.h"

/**
 * Command to set IntakeSubsystem power.
 */
class SetIntakePower : public frc2::CommandHelper<frc2::Command, SetIntakePower> {
 public:
  explicit SetIntakePower(double newPower, IntakeSubsystem* intakeRef);

  void Initialize() override;

  bool IsFinished() override;

 private:
    IntakeSubsystem *intake;
    double power = 0.0;
};