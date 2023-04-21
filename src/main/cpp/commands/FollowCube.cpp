#include "commands/FollowCube.h"

FollowCube::FollowCube(DriveSubsystem* driveRef, IntakeSubsystem* intakeRef, LimelightSubsystem* limeRef) 
: drive(driveRef),
intake(intakeRef),
limelight(limeRef) {
  AddRequirements({driveRef, intakeRef, limeRef});
  SetName("FollowCube");
}

void FollowCube::Initialize() {
  drive->SetLimiting(false);
  if(!limelight->IsTarget()) done = true;
  xController.Reset();
  xController.SetSetpoint(0.0);
  thetaController.Reset();
  thetaController.SetSetpoint(0.0);
  intake->SetPower(1.0);
}

void FollowCube::Execute() {
  double currentX = limelight->GetXOffset();
  double currentSize = limelight->GetTargetArea();
  double x = xController.Calculate(currentX);
  double theta = thetaController.Calculate(currentSize);
  drive->Drive(units::meters_per_second_t{x}, 0.0_mps, units::degrees_per_second_t{theta}, false);

}

void FollowCube::End(bool interrupted) {
  drive->Drive(0.0_mps, 0.0_mps, 0_deg_per_s, false);
  if(cubeCaught) intake->SetPower(0.12);
}

bool FollowCube::IsFinished() {
  return done;
}