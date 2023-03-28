#include "commands/SetIntakePower.h"

SetIntakePower::SetIntakePower(double newPower, IntakeSubsystem *intakeRef) 
: intake(intakeRef) {
  AddRequirements(intakeRef);
  power = newPower;
}

void SetIntakePower::Initialize() {
  intake->SetState(IntakeConstants::kPowerMode);
  intake->SetPower(power);
}

bool SetIntakePower::IsFinished() {
  return true;
}