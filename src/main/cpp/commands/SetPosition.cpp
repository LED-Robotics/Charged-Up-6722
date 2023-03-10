#include "commands/SetPosition.h"

SetPosition::SetPosition(int position, ElevatorSubsystem* elevRef, ArmSubsystem* armRef, IntakeSubsystem* intakeRef) 
: elevator(elevRef),
arm(armRef),
intake(intakeRef) {
  AddRequirements({elevRef, armRef, intakeRef});
  target = position;
}

void SetPosition::Initialize() {
    if(target == 0) {
      elevator->SetTargetPosition(ElevatorConstants::kStartPosition);
      intake->SetPosition(IntakeConstants::kStartPosition);
    } else if(target == 1) {
      elevator->SetTargetPosition(ElevatorConstants::kFloorPickupPosition);
      intake->SetPosition(IntakeConstants::kFloorPickupPosition);
    } else if(target == 2) {
      elevator->SetTargetPosition(ElevatorConstants::kMidDropoffPosition);
      arm->SetTargetPosition(ArmConstants::kMidDropoffPosition);
      intake->SetPosition(IntakeConstants::kMidDropoffPosition);
    } else if(target == 3) {
      elevator->SetTargetPosition(ElevatorConstants::kHighDropoffPosition);
      arm->SetTargetPosition(ArmConstants::kHighDropoffPosition);
      intake->SetPosition(IntakeConstants::kHighDropoffPosition);
    }
}

void SetPosition::Execute() {
  if(target == 0 && elevator->IsAtTarget() && intake->IsAtTarget()) {
    elevator->SetTargetPosition(ElevatorConstants::kStartPosition);
    arm->SetTargetPosition(ArmConstants::kStartPosition);
  } else if(target == 1 && elevator->IsAtTarget() && intake->IsAtTarget()) {
    arm->SetTargetPosition(ArmConstants::kFloorPickupPosition);
    elevator->SetTargetPosition(ElevatorConstants::kFloorPickupPosition);
  }
}

bool SetPosition::IsFinished() {
  return elevator->IsAtTarget() && arm->IsAtTarget() && intake->IsAtTarget();
}