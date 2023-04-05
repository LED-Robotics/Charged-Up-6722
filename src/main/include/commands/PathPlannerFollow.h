#pragma once
#include "Constants.h"
#include <iostream>
#include <memory>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc/Timer.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>



#include "subsystems/DriveSubsystem.h"

using namespace pathplanner;

/**
 * A command that sets the position of the elevator, intake, and arm subsystems. 
 * This command controls the order in which the subsystems move based on the target position,
 * as opposed to the original method of moving all three simultaneously all the time.
 */
class PathPlannerFollow : public frc2::CommandHelper<frc2::SequentialCommandGroup, PathPlannerFollow> {
 public:
  explicit PathPlannerFollow(DriveSubsystem* driveRef);
};