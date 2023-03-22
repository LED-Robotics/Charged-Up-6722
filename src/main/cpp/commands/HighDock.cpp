#include "commands/HighDock.h"

HighDock::HighDock(DriveSubsystem *drive, ElevatorSubsystem *elev, ArmSubsystem *arm, IntakeSubsystem *intake) {
  AddCommands(
    frc2::InstantCommand(
      [=]() { 
        drive->Drive(0.7_mps, 0_mps, 0_deg_per_s, false);
      }, {drive}),
    frc2::WaitCommand(0.7_s),
    frc2::InstantCommand(
      [=]() { 
        drive->Drive(0.0_mps, 0_mps, 0_deg_per_s, false);
      }, {drive}),

    SetPosition(2, elev, arm, intake),

    frc2::InstantCommand(
      [=]() { 
        drive->Drive(-0.7_mps, 0_mps, 0_deg_per_s, false);
      }, {drive}),
    frc2::WaitCommand(0.7_s),
    frc2::InstantCommand(
      [=]() { 
        drive->Drive(0.0_mps, 0_mps, 0_deg_per_s, false);
      }, {drive}),
    SetPosition(3, elev, arm, intake),
    frc2::InstantCommand(
      [=]() { 
        intake->SetState(IntakeConstants::kPowerMode);
      }, {elev, intake}),
      frc2::InstantCommand(
      [=]() { 
        intake->SetPower(0.12);
      }, {intake}),
      frc2::WaitCommand(0.1_s),
      frc2::InstantCommand(
      [=]() { 
        intake->SetPower(-1.0);
      }, {intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand(
      [=]() { 
        intake->SetPower(0.0);
      }, {intake}),
      SetPosition(2, elev, arm, intake),
      frc2::InstantCommand(
      [=]() { 
        drive->Drive(0.7_mps, 0_mps, 0_deg_per_s, false);
      }, {drive}),
      frc2::WaitCommand(0.7_s),
      frc2::InstantCommand(
      [=]() { 
        drive->Drive(0.0_mps, 0_mps, 0_deg_per_s, false);
      }, {drive}),
      SetPosition(0, elev, arm, intake),
      frc2::FunctionalCommand(
      [=] { ; },
      [=] { drive->Drive(2.5_mps, 0_mps, 0_deg_per_s, false); },
      [=] (bool interrupted) { ; },
      [=] { return drive->GetPitch() > 6.0; },
      {drive}),
      frc2::FunctionalCommand(
      [=] { ; },
      [=] { drive->Drive(2.5_mps, 0_mps, 0_deg_per_s, false); },
      [=] (bool interrupted) { drive->Drive(0.7_mps, 0_mps, 0_deg_per_s, false); },
      [=] { return drive->GetPitch() < 2.0; },
      {drive}),
      frc2::WaitCommand(1.5_s),
      frc2::InstantCommand(
      [=]() { 
        drive->Drive(0.0_mps, 0_mps, 0_deg_per_s, false);
      }, {drive}),
      GyroDock(2.0, drive)
  );
}