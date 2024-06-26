#pragma once

#include "Constants.h"
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>


/**
 * Command that sets the position of the elevator, intake, and arm subsystems. 
 * This Command controls the order in which the subsystems move based on the target position,
 * as opposed to the original method of moving all three simultaneously all the time.
 */
class SetPosition : public frc2::CommandHelper<frc2::Command, SetPosition> {
 public:
  explicit SetPosition(int position, ElevatorSubsystem* elevRef, ArmSubsystem* armRef, IntakeSubsystem* intakeRef);

  void Initialize() override;

  void Execute() override;

  bool IsFinished() override;

 private:
    bool isCone = true;
    int target = 0;
    ElevatorSubsystem *elevator;
    ArmSubsystem *arm;
    IntakeSubsystem *intake;
};