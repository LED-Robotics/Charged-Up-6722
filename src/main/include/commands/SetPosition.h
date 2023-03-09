#pragma once

#include "Constants.h"
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

/**
 * A simple command that grabs a hatch with the HatchSubsystem.  Written
 * explicitly for pedagogical purposes.  Actual code should inline a command
 * this simple with InstantCommand.
 *
 * @see InstantCommand
 */
class SetPosition : public frc2::CommandHelper<frc2::CommandBase, SetPosition> {
 public:
  explicit SetPosition(int position, ElevatorSubsystem* elevRef, ArmSubsystem* armRef, IntakeSubsystem* intakeRef);

  void Initialize() override;

  void Execute() override;

  bool IsFinished() override;

 private:
    int target = 0;
    ElevatorSubsystem* elevator;
    ArmSubsystem* arm;
    IntakeSubsystem* intake;
};