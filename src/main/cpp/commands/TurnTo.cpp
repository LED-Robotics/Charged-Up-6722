#include "commands/TurnTo.h"

TurnTo::TurnTo(double target, DriveSubsystem *driveRef) 
: drive(driveRef) {
  AddRequirements(driveRef);
  angle = target;
}

void TurnTo::Initialize() {
  pidController.SetTolerance(12.0);
  pidController.SetSetpoint(angle);
  double initial = (double)drive->GetPose().Rotation().Degrees();
  if(abs(angle - initial) < 360.0 - angle) flipped = true;
}

void TurnTo::Execute() {
  double current = (double)drive->GetPose().Rotation().Degrees();
  double value = pidController.Calculate(current);
  drive->Drive(0.0_mps, 0.0_mps, units::degrees_per_second_t{-abs(value)}, false);
}

void TurnTo::End(bool interrupted) {
  drive->Drive(0.0_mps, 0.0_mps, 0.0_deg_per_s, false);
}


bool TurnTo::IsFinished() {
  return pidController.AtSetpoint();
}