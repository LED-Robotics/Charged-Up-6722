#include "commands/WaitDrive.h"

WaitDrive::WaitDrive(units::meter_t distance, units::velocity::meters_per_second_t speed, DriveSubsystem* driveRef) 
: drive(driveRef) {
  AddRequirements(drive);
  target = distance;
  targetSpeed = speed;
  // find the time it would take to move the target distance at the target speed
  totalTime = units::second_t{abs((double)distance / (double)speed)};
}

void WaitDrive::Initialize() {
  // start moving at target speed
  drive->Drive(targetSpeed, 0_mps, 0_deg_per_s, false);
  // start timer
  doneTimer.Start();
}

void WaitDrive::Execute() {
  // keep driving at target speed
  drive->Drive(targetSpeed, 0_mps, 0_deg_per_s, false);
}

void WaitDrive::End(bool interrupted) {
  // stop driving
  drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
}

bool WaitDrive::IsFinished() {
  // finished when time elapses
  bool done = doneTimer.HasElapsed(totalTime);
  if(done) doneTimer.Stop();
  return done;
}