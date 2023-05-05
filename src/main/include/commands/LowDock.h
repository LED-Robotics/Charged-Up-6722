#pragma once

#include "Constants.h"
#include <iostream>
#include "commands/SetPosition.h"
#include "commands/SetIntakePower.h"
#include "commands/ToPoint.h"
#include "commands/WaitDrive.h"
#include "commands/GyroDock.h"
#include <frc2/command/CommandBase.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

/**
 * Auton Command that gets a low piece, mobility, and dock/engage.
 */
class LowDock : public frc2::CommandHelper<frc2::SequentialCommandGroup, LowDock> {
 public:
    LowDock(DriveSubsystem *driveRef, ElevatorSubsystem* elevRef, ArmSubsystem* armRef, IntakeSubsystem* intakeRef);
};