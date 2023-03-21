#pragma once

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

void setPosition(int state, ElevatorSubsystem *elevator, ArmSubsystem *arm, IntakeSubsystem *intake);

bool gyroDock(bool forward, DriveSubsystem *drive);