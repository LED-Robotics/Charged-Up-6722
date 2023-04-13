#pragma once

#include "Constants.h"
#include <iostream>
#include "commands/SetPosition.h"
#include "commands/SetIntakePower.h"
#include "commands/TrajectoryRelative.h"
#include "commands/WaitDrive.h"
#include "commands/ToPoint.h"
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
 * A command that sets the position of the elevator, intake, and arm subsystems. 
 * This command controls the order in which the subsystems move based on the target position,
 * as opposed to the original method of moving all three simultaneously all the time.
 */
class HighDock : public frc2::CommandHelper<frc2::SequentialCommandGroup, HighDock> {
 public:
    HighDock(DriveSubsystem *driveRef, ElevatorSubsystem* elevRef, ArmSubsystem* armRef, IntakeSubsystem* intakeRef);
};