#include "commands/GyroDock.h"

GyroDock::GyroDock(double targetSpeed, DriveSubsystem *driveRef) 
: drive(driveRef) {
  AddRequirements(driveRef);
  speed = -targetSpeed;
  if(targetSpeed < 0.0) backwards = true;
}

void GyroDock::Initialize() {
  initialPitch = drive->GetPitch();
  double current = drive->GetPitch();
  if(!backwards) {
    while(current > initialPitch - range) {
      current = drive->GetPitch();
      drive->Drive(units::meters_per_second_t{speed}, 0_mps, 0_deg_per_s, false);
    };
  } else {
    while(current < initialPitch + range) {
      current = drive->GetPitch();
      drive->Drive(units::meters_per_second_t{speed}, 0_mps, 0_deg_per_s, false);
    };
  }
  
  range = initialPitch - current;
}

void GyroDock::Execute() {
  double current = drive->GetPitch();
  double error = (initialPitch - current) / range * kP;
  if(current > initialPitch && !backwards) tippedForward = true;
  else if(current > initialPitch && backwards) tippedForward = true;
  // std::cout << "Timer Time: " << (double)successTimer.Get() << '\n';
  if(current > initialPitch + (changeThreshold / 2) && current < initialPitch - (changeThreshold / 2)) {
    drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
    if(successTimer.Get() == 0_s) successTimer.Start();

  } else {
    if(successTimer.Get() != 0_s) {
      successTimer.Stop();
      successTimer.Reset();
    }
    drive->Drive(units::meters_per_second_t{(tippedForward ? speed*0.5 : speed) * error}, 0_mps, 0_deg_per_s, false);
  }
}

bool GyroDock::IsFinished() {
  return successTimer.HasElapsed(successTime);
}