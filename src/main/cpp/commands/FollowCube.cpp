#include "commands/FollowCube.h"

FollowCube::FollowCube(DriveSubsystem* driveRef, IntakeSubsystem* intakeRef, LimelightSubsystem* limeRef) 
: drive(driveRef),
intake(intakeRef),
limelight(limeRef) {
  AddRequirements({driveRef, intakeRef, limeRef});
  SetName("FollowCube");
}

void FollowCube::Initialize() {
  // configure drivetrain for following
  drive->SetLimiting(false);
  if(!limelight->IsTarget()) done = true; // set flag to end if no cube is visible
  xController.Reset();
  xController.SetSetpoint(0.0);
  thetaController.Reset();
  thetaController.SetSetpoint(0.0);
  intake->SetPower(1.0);  // turn on intake
}

void FollowCube::Execute() {
  // use PID to move to cube
  double currentX = limelight->GetXOffset();
  double currentSize = limelight->GetTargetArea();
  double x = xController.Calculate(currentX);
  double theta = thetaController.Calculate(currentSize);
  drive->Drive(units::meters_per_second_t{x}, 0.0_mps, units::degrees_per_second_t{theta}, false);
  // FUNCTION IS UNFINISHED
  // NEED TO ADD CODE TO DETECT X/Y POSES AND SIZE OF TARGET THAT WE CALL "DONE"
  // AS OF RIGHT NOW THIS COMMAND NEVER ENDS
}

void FollowCube::End(bool interrupted) {
  // stop driving
  drive->Drive(0.0_mps, 0.0_mps, 0_deg_per_s, false);
  // set intake to holding
  if(cubeCaught) intake->SetPower(0.12);
}

bool FollowCube::IsFinished() {
  return done;
}