#include "commands/LowPlaceThenBreak.h"

LowPlaceThenBreak::LowPlaceThenBreak(DriveSubsystem *drive, ElevatorSubsystem *elev, ArmSubsystem *arm, IntakeSubsystem *intake) {
  SetName("Low Place Then Break");
  AddCommands(
    
    SetIntakePower(0.12, intake),       // set intake to holding power
    
    SetPosition(4, elev, arm, intake),  // extend to floor standing
        
    SetIntakePower(-1.0, intake),       // intake full reverse to spit out cone
    
    frc2::WaitCommand(0.6_s),           // wait until cone is out

    SetPosition(0, elev, arm, intake),  // retract to stored
    
    SetIntakePower(0.0, intake),        // turn off intake

    WaitDrive(3.8_m, 1.0_mps, drive)
    
  );
}