#include "commands/GyroDock.h"

GyroDock::GyroDock(double targetSpeed, DriveSubsystem *driveRef) 
: drive(driveRef) {
  AddRequirements(driveRef);
  SetName("Gyro Dock");
  speed = -targetSpeed;
  if(targetSpeed < 0.0) backwards = true;
}

void GyroDock::Initialize() {
  // turn off SlewRateLimiters
  drive->SetLimiting(false);
  // get pitch when the robot is level on the floor
  initialPitch = drive->GetPitch();
}

void GyroDock::Execute() {
  double current = drive->GetPitch(); // current pitch
  if(!initialApproachCompleted) {
    // move until robot tips up
    if(backwards) { // invert
      if(current > initialPitch - range) {
        drive->Drive(units::meters_per_second_t{speed}, 0_mps, 0_deg_per_s, false);
      } else {
        range = initialPitch - current; // set max angle range
        initialApproachCompleted = true;
      }
    } else {
      if(current < initialPitch + range) {
        drive->Drive(units::meters_per_second_t{speed}, 0_mps, 0_deg_per_s, false);
      } else {
        range = initialPitch - current; // set max angle range
        initialApproachCompleted = true;
      }
    }
    
  } else {
    // balance after initial approach
    double error = (initialPitch - current) / range * kP;
    // flag if the robot tipped too far
    if(current < initialPitch && !backwards) tippedForward = true;
    else if(current > initialPitch && backwards) tippedForward = true;
    if(current > initialPitch + (changeThreshold / 2) && current < initialPitch - (changeThreshold / 2)) {
      // stop the robot if balanced
      drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
    } else {
      // drive adjusting speed with PID. speed is multiplied by 0.43 if the bot tips too far.
      drive->Drive(units::meters_per_second_t{(tippedForward ? speed*0.43 : speed) * error}, 0_mps, 0_deg_per_s, false);
    }
  }
}

bool GyroDock::IsFinished() {
  // finish when cancelled
  return false;
}