#include "commands/SetPosition.h"

SetPosition::SetPosition(int position, ElevatorSubsystem *elevRef, ArmSubsystem *armRef, IntakeSubsystem *intakeRef) 
: elevator(elevRef),
arm(armRef),
intake(intakeRef) {
  AddRequirements({elevRef, armRef, intakeRef});
  target = position;
}

void SetPosition::Initialize() {
    // SmartDashboard::PutBoolean("coneInSet", isCone);  // print to Shuffleboard
    isCone = arm->GetConeMode();
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
      elevator->SetTargetPosition(isCone ? ElevatorConstants::kMidDropoffPosition : ElevatorConstants::kCubePosition);
      arm->SetTargetAngle(isCone ? ArmConstants::kMidDropoffAngle : ArmConstants::kMidCubeAngle);
      intake->SetWristState(IntakeConstants::kPositionMode);
      intake->SetTargetPosition(isCone ? IntakeConstants::kMidDropoffPosition : IntakeConstants::kMidCubePosition);
    } else if(target == 3) {
      arm->SetTargetAngle(isCone ? ArmConstants::kHighDropoffAngle : ArmConstants::kHighCubeAngle);
      intake->SetWristState(IntakeConstants::kPositionMode);
      intake->SetTargetPosition(isCone ? IntakeConstants::kHighDropoffPosition : IntakeConstants::kHighCubePosition);
    } else if(target == 4) {
      elevator->SetTargetPosition(ElevatorConstants::kFloorStandingPickupPosition);
      arm->SetTargetAngle(ArmConstants::kFloorStandingPickupAngle);
      intake->SetWristState(IntakeConstants::kPositionMode);
      intake->SetTargetPosition(IntakeConstants::kFloorStandingPickupPosition);
    } else if(target == 5) {
      elevator->SetTargetPosition(300);
      arm->SetTargetAngle(ArmConstants::kAutonStart);
    } else if(target == 6) {
      arm->SetTargetAngle(ArmConstants::kDoubleStation);
      intake->SetWristState(IntakeConstants::kPositionMode);
      intake->SetTargetPosition(IntakeConstants::kDoubleStation);
    }
}

void SetPosition::Execute() {
  if(target == 0 && elevator->IsAtTarget()) {
    arm->SetTargetAngle(ArmConstants::kStartAngle);
  } else if(target == 1 && elevator->IsAtTarget() && intake->IsAtTarget()) {
    arm->SetTargetAngle(ArmConstants::kFloorPickupAngle);
    elevator->SetTargetPosition(ElevatorConstants::kFloorPickupPosition);
  } else if(target == 3 && arm->IsAtTarget() && intake->IsAtTarget()) {
    elevator->SetTargetPosition(isCone ? ElevatorConstants::kHighDropoffPosition : ElevatorConstants::kCubePosition);
  } else if(target == 5 && elevator->IsAtTarget() && arm->IsAtTarget()) {
    intake->SetWristState(IntakeConstants::kPositionMode);
    intake->SetTargetPosition(IntakeConstants::kAutonStart);
  } else if(target == 6 && arm->IsAtTarget() && intake->IsAtTarget()) {
    elevator->SetTargetPosition(ElevatorConstants::kDoubleStation);
  }
}

bool SetPosition::IsFinished() {
  return elevator->IsAtTarget() && arm->IsAtTarget() && intake->IsAtTarget();
}