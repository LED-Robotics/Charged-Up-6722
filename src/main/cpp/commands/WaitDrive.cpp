#include "commands/WaitDrive.h"

WaitDrive::WaitDrive(units::meter_t distance, units::velocity::meters_per_second_t speed, DriveSubsystem* driveRef) 
: drive(driveRef) {
  AddRequirements(drive);
  target = distance;
  targetSpeed = speed;
  totalTime = units::second_t{abs((double)distance / (double)speed)};
}

void WaitDrive::Initialize() {
  drive->Drive(targetSpeed, 0_mps, 0_deg_per_s, false);
  doneTimer.Start();
}

void WaitDrive::Execute() {
  drive->Drive(targetSpeed, 0_mps, 0_deg_per_s, false);
}

void WaitDrive::End(bool interrupted) {
  drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
}

bool WaitDrive::IsFinished() {
  bool done = doneTimer.HasElapsed(totalTime);
  if(done) doneTimer.Stop();
  return done;
}