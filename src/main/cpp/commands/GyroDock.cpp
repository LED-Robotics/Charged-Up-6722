#include "commands/GyroDock.h"

GyroDock::GyroDock(double targetSpeed, DriveSubsystem *driveRef) 
: drive(driveRef) {
  AddRequirements(driveRef);
  SetName("Gyro Dock");
  speed = -targetSpeed;
  if(targetSpeed < 0.0) backwards = true;
}

void GyroDock::Initialize() {
  initialPitch = drive->GetPitch();
}

void GyroDock::Execute() {
  double current = drive->GetPitch();
  if(!initialApproachCompleted) {
    if(!backwards) {
      if(current > initialPitch - range) {
        drive->Drive(units::meters_per_second_t{speed}, 0_mps, 0_deg_per_s, false);
      } else {
        range = initialPitch - current;
        initialApproachCompleted = true;
      }
    } else {
      if(current < initialPitch + range) {
        drive->Drive(units::meters_per_second_t{speed}, 0_mps, 0_deg_per_s, false);
      } else {
        range = initialPitch - current;
        initialApproachCompleted = true;
      }
    }
    
  } else {
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
      drive->Drive(units::meters_per_second_t{(tippedForward ? speed*0.50 : speed) * error}, 0_mps, 0_deg_per_s, false);
    }
  }
}

bool GyroDock::IsFinished() {
  return successTimer.HasElapsed(successTime);
}