#include "commands/GyroDock.h"

GyroDock::GyroDock(double targetSpeed, DriveSubsystem* driveRef) 
: drive(driveRef) {
  AddRequirements(driveRef);
  speed = targetSpeed;
}

void GyroDock::Initialize() {
}

void GyroDock::Execute() {
}

bool GyroDock::IsFinished() {
  return true;
}