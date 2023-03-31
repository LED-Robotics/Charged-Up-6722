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
      arm->SetTargetAngle(ArmConstants::kMidDropoffAngle);
      intake->SetWristState(IntakeConstants::kPositionMode);
      intake->SetTargetPosition(IntakeConstants::kMidDropoffPosition);
    } else if(target == 3) {
      arm->SetTargetAngle(ArmConstants::kHighDropoffAngle);
      intake->SetWristState(IntakeConstants::kPositionMode);
      intake->SetTargetPosition(IntakeConstants::kHighDropoffPosition);
    } else if(target == 4) {
      elevator->SetTargetPosition(ElevatorConstants::kFloorStandingPickupPosition);
      arm->SetTargetAngle(ArmConstants::kFloorStandingPickupAngle);
      intake->SetWristState(IntakeConstants::kPositionMode);
      intake->SetTargetPosition(IntakeConstants::kFloorStandingPickupPosition);
    } else if(target == 5) {
      elevator->SetTargetPosition(300);
      arm->SetTargetAngle(ArmConstants::kAutonStart);
    }
}

void SetPosition::Execute() {
  if(target == 0 && elevator->IsAtTarget()) {
    arm->SetTargetAngle(ArmConstants::kStartAngle);
  } else if(target == 1 && elevator->IsAtTarget() && intake->IsAtTarget()) {
    arm->SetTargetAngle(ArmConstants::kFloorPickupAngle);
    elevator->SetTargetPosition(ElevatorConstants::kFloorPickupPosition);
  } else if(target == 3 && arm->IsAtTarget() && intake->IsAtTarget()) {
    elevator->SetTargetPosition(ElevatorConstants::kHighDropoffPosition);
  } else if(target == 5 && elevator->IsAtTarget() && arm->IsAtTarget()) {
    intake->SetWristState(IntakeConstants::kPositionMode);
    intake->SetTargetPosition(IntakeConstants::kAutonStart);
  }
}

bool SetPosition::IsFinished() {
  return elevator->IsAtTarget() && arm->IsAtTarget() && intake->IsAtTarget();
}