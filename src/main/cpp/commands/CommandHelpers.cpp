#include "commands/CommandHelpers.h"

void setPosition(int target, ElevatorSubsystem *elevator, ArmSubsystem *arm, IntakeSubsystem *intake) {
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
  }
  if(target == 0) {
    while(!elevator->IsAtTarget());
    arm->SetTargetAngle(ArmConstants::kStartAngle);
  } else if(target == 1) {
    while(!elevator->IsAtTarget() && intake->IsAtTarget());
    arm->SetTargetAngle(ArmConstants::kFloorPickupAngle);
    elevator->SetTargetPosition(ElevatorConstants::kFloorPickupPosition);
  } else if(target == 3) {
    while(!arm->IsAtTarget() && intake->IsAtTarget());
    elevator->SetTargetPosition(ElevatorConstants::kHighDropoffPosition);
  }
}

bool gyroDock(bool forward, DriveSubsystem *drive) {
}