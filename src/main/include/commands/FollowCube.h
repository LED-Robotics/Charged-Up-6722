#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LimelightSubsystem.h"

/**
 * A command that sets the position of the elevator, intake, and arm subsystems. 
 * This command controls the order in which the subsystems move based on the target position,
 * as opposed to the original method of moving all three simultaneously all the time.
 */
class FollowCube : public frc2::CommandHelper<frc2::CommandBase, FollowCube> {
 public:
  explicit FollowCube(DriveSubsystem* driveRef, IntakeSubsystem* intakeRef, LimelightSubsystem* limeRef);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
    bool cubeCaught = false;
    bool done = false;
    frc2::PIDController xController{2.5, 0.0, 0.0};
    frc2::PIDController thetaController{-0.07, 0.0, 0.0};
    DriveSubsystem *drive;
    IntakeSubsystem *intake;
    LimelightSubsystem *limelight;
};