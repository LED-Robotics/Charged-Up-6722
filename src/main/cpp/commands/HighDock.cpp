#include "commands/HighDock.h"

HighDock::HighDock(DriveSubsystem *driveRef, ElevatorSubsystem *elevRef, ArmSubsystem *armRef, IntakeSubsystem *intakeRef) 
: drive(driveRef),
elevator(elevRef),
arm(armRef),
intake(intakeRef) {
  AddRequirements({driveRef, elevRef, armRef, intakeRef});
}

void HighDock::Initialize() {
  intake->SetState(IntakeConstants::kPowerMode);
  intake->SetPower(0.12);
  elevator->SetTargetPosition(50000);
  while(!elevator->IsAtTarget());
  intake->SetPower(-1.0);
  Wait(1.0_s);
  while(drive->GetPitch() < 8.0) {
    drive->Drive(-1.5_mps, 0.0_mps, 0_deg_per_s, false);
  }
}

void HighDock::Execute() {
}

bool HighDock::IsFinished() {
}