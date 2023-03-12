#include "commands/SetPosition.h"

SetPosition::SetPosition(int position, ElevatorSubsystem *elevRef, ArmSubsystem *armRef, IntakeSubsystem *intakeRef) 
: elevator(elevRef),
arm(armRef),
intake(intakeRef) {
  AddRequirements({elevRef, armRef, intakeRef});
  target = position;
}

void SetPosition::Initialize() {
    if(target == 0) {
      elevator->SetTargetPosition(ElevatorConstants::kStartPosition);
      intake->SetWristState(IntakeConstants::kPositionMode);
      intake->SetTargetPosition(IntakeConstants::kStartPosition);
    } else if(target == 1) {
      elevator->SetTargetPosition(ElevatorConstants::kFloorPickupPosition);
      arm->SetTargetAngle(ArmConstants::kFloorPickupAngle);
      intake->SetWristState(IntakeConstants::kPositionMode);
      intake->SetTargetPosition(IntakeConstants::kFloorPickupPosition);
    } else if(target == 2) {
      elevator->SetTargetPosition(ElevatorConstants::kMidDropoffPosition);
      // arm->SetTargetPosition(ArmConstants::kMidDropoffPosition);
      arm->SetTargetAngle(ArmConstants::kMidDropoffAngle);
      intake->SetWristState(IntakeConstants::kPositionMode);
      intake->SetTargetPosition(IntakeConstants::kMidDropoffPosition);
    } else if(target == 3) {
      // arm->SetTargetPosition(ArmConstants::kHighDropoffPosition);
      arm->SetTargetAngle(ArmConstants::kHighDropoffAngle);
      intake->SetWristState(IntakeConstants::kPositionMode);
      intake->SetTargetPosition(IntakeConstants::kHighDropoffPosition);
    } else if(target == 4) {
      elevator->SetTargetPosition(ElevatorConstants::kFloorStandingPickupPosition);
      arm->SetTargetAngle(ArmConstants::kFloorStandingPickupAngle);
      intake->SetWristState(IntakeConstants::kPositionMode);
      intake->SetTargetPosition(IntakeConstants::kFloorStandingPickupPosition);
    }
}

void SetPosition::Execute() {
  if(target == 0 && elevator->IsAtTarget()) {
    // elevator->SetTargetPosition(ElevatorConstants::kStartPosition);
    // arm->SetTargetPosition(ArmConstants::kStartPosition);
    arm->SetTargetAngle(ArmConstants::kStartAngle);
  } else if(target == 1 && elevator->IsAtTarget() && intake->IsAtTarget()) {
    // arm->SetTargetPosition(ArmConstants::kFloorPickupPosition);
    arm->SetTargetAngle(ArmConstants::kFloorPickupAngle);
    elevator->SetTargetPosition(ElevatorConstants::kFloorPickupPosition);
  } else if(target == 3 && arm->IsAtTarget() && intake->IsAtTarget()) {
    elevator->SetTargetPosition(ElevatorConstants::kHighDropoffPosition);
  }
}

bool SetPosition::IsFinished() {
  return elevator->IsAtTarget() && arm->IsAtTarget() && intake->IsAtTarget();
}