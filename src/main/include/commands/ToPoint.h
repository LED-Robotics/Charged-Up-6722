#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/DriveSubsystem.h"

/**
 * Command that moves the robot to a set Pose2d based on its odometry.
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