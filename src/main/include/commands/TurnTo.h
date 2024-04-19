#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include <frc/controller/PIDController.h>

#include "subsystems/DriveSubsystem.h"

/**
 * Command that turns the robot to an absolute angle based on the field.
 */
class TurnTo : public frc2::CommandHelper<frc2::Command, TurnTo> {
 public:
  explicit TurnTo(double target, DriveSubsystem* driveRef);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
    double angle = 0.0;
    bool flipped = false;
    const double kP = 1.4;
    frc::PIDController pidController{kP, 0.0, 0.0};
    DriveSubsystem *drive;
};