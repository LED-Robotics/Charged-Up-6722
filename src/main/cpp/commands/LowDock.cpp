#include "commands/LowDock.h"

LowDock::LowDock(DriveSubsystem *drive, ElevatorSubsystem *elev, ArmSubsystem *arm, IntakeSubsystem *intake) {
  SetName("Low Dock");
  AddCommands(
    SetIntakePower(0.12, intake),       // set intake to holding power
    
    SetPosition(4, elev, arm, intake),  // extend to standing position
        
    SetIntakePower(-1.0, intake),       // intake full reverse to spit out cone
    
    frc2::WaitCommand(0.6_s),           // wait until cone is out

    SetPosition(0, elev, arm, intake),  // retract to stored
    
    SetIntakePower(0.0, intake),        // turn off intake

    frc2::FunctionalCommand(            // drive until robot hits charge station
      [=] { ; },
      [=] { drive->Drive(-2.0_mps, 0_mps, 0_deg_per_s, false); },
      [=] (bool interrupted) { ; },
      [=] { return drive->GetPitch() > 6.0; },
      {drive}),

    frc2::FunctionalCommand(            // drive until robot tips down and then slow down
      [=] { ; },
      [=] { drive->Drive(-1.3_mps, 0_mps, 0_deg_per_s, false); },
      [=] (bool interrupted) { drive->Drive(0.0_mps, 0_mps, 0_deg_per_s, false); },
      [=] { return drive->GetPitch() < -6.0; },
      {drive}),

    ToPoint({5.5_m, 0.0_m, {180.0_deg}}, drive),   // drive back from link station

    frc2::WaitCommand(0.5_s),         // drive for another extra bit to make sure we break the auto line
    
    frc2::InstantCommand(             // stop the drive
      [=]() { 
        drive->Drive(0.0_mps, 0_mps, 0_deg_per_s, false);
      }, {drive}),
    
    GyroDock(-1.8, drive)              // dock on charge station
  );
}