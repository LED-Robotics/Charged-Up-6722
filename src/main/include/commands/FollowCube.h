#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LimelightSubsystem.h"

/**
 * A Command meant to follow and intake a cube using the Limelight
 */
class FollowCube : public frc2::CommandHelper<frc2::Command, FollowCube> {
 public:
  explicit FollowCube(DriveSubsystem* driveRef, IntakeSubsystem* intakeRef, LimelightSubsystem* limeRef);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
    bool cubeCaught = false;
    bool done = false;
    frc::PIDController xController{2.5, 0.0, 0.0};
    frc::PIDController thetaController{-0.07, 0.0, 0.0};
    DriveSubsystem *drive;
    IntakeSubsystem *intake;
    LimelightSubsystem *limelight;
};