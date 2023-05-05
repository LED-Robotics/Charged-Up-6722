#include "commands/ToPoint.h"

ToPoint::ToPoint(frc::Pose2d target, DriveSubsystem *driveRef) 
: targetPose(target),
drive(driveRef),
targetX{(double)target.X()},
targetY{(double)target.Y()},
targetDeg{(double)target.Rotation().Degrees()} {
  AddRequirements(driveRef);
  SetName("To Point");
}

void ToPoint::Initialize() {
  // configure DriveSubsystem to hold at target Pose2d
  drive->SetLimiting(false);
  drive->SetPoseToHold(targetPose);
  drive->StartHolding();
}

void ToPoint::Execute() {
  // move drivetrain according to its CalculateHolding method
  frc::Pose2d current = drive->GetPose();
  currentX = (double)current.X();
  currentY = (double)current.Y();
  currentDeg = (double)current.Rotation().Degrees();
  currentDeg = SwerveModule::PlaceInAppropriate0To360Scope(targetDeg, currentDeg);
  auto speeds = drive->CalculateHolding();
  drive->Drive(speeds.vx, speeds.vy, {speeds.omega}, true);
}

void ToPoint::End(bool interrupted) {
  // stop moving
  drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
}

bool ToPoint::IsFinished() {
  // end when drivetrain is within deadzones on all three Pose2d components
  bool xMet = currentX > targetX - (translationDeadzone / 2) && currentX < targetX + (translationDeadzone / 2);
  bool yMet = currentY > targetY - (translationDeadzone / 2) && currentY < targetY + (translationDeadzone / 2);
  bool degMet = currentDeg > targetDeg - (rotationDeadzone / 2) && currentDeg < targetDeg + (rotationDeadzone / 2);
  return xMet && yMet && degMet;
}