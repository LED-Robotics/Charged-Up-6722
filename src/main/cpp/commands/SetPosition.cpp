#include "commands/SetPosition.h"

SetPosition::SetPosition(int position, ElevatorSubsystem* elevRef, ArmSubsystem* armRef, IntakeSubsystem* intakeRef) 
: elevator(elevRef),
arm(armRef),
intake(intakeRef) {
  AddRequirements({elevRef, armRef, intakeRef});
  target = position;
}

void SetPosition::Initialize() {
    switch(target) {
        case 0:
            break;
        case 1:
            break;
        case 2:
            break;
    }
}

void SetPosition::Execute() {
}

bool SetPosition::IsFinished() {
  return true;
}