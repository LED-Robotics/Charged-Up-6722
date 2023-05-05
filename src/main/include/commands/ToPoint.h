#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/DriveSubsystem.h"

/**
 * A command that sets the position of the elevator, intake, and arm subsystems. 
 * This command controls the order in which the subsystems move based on the target position,
 * as opposed to the original method of moving all three simultaneously all the time.
 */
class ToPoint : public frc2::CommandHelper<frc2::CommandBase, ToPoint> {
 public:
  explicit ToPoint(frc::Pose2d target, DriveSubsystem* driveRef);

  void Initialize() override;

  void Execute() override;
  
  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
    frc::Pose2d targetPose;
    DriveSubsystem *drive;
    double targetX;
    double targetY;
    double targetDeg;
    double translationDeadzone = 0.2;
    double rotationDeadzone = 14.0;
    double currentX = 0.0;
    double currentY = 0.0;
    double currentDeg = 0.0;
};