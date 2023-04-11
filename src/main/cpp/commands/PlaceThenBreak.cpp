#include "commands/PlaceThenBreak.h"

PlaceThenBreak::PlaceThenBreak(DriveSubsystem *drive, ElevatorSubsystem *elev, ArmSubsystem *arm, IntakeSubsystem *intake) {
  SetName("Place Then Break");
  AddCommands(
    SetIntakePower(0.12, intake),       // set intake to holding power

    frc2::ParallelCommandGroup(   // perform simultaneously
      WaitDrive(0.5_m, 0.8_mps, drive),   // drive back from link station

      SetPosition(3, elev, arm, intake)  // extent to mid
    ),

    WaitDrive(0.5_m, -0.8_mps, drive),  // drive to link station
    
    // SetPosition(3, elev, arm, intake),  // extent to high
        
    SetIntakePower(-1.0, intake),       // intake full reverse to spit out cone
    
    frc2::WaitCommand(0.6_s),           // wait until cone is out
    
    SetIntakePower(0.0, intake),        // turn off intake

    WaitDrive(0.5_m, 0.5_mps, drive),   // drive back from link station
    
    SetPosition(2, elev, arm, intake),  // retract elevator to mid    
    
    frc2::ParallelCommandGroup(   // perform simultaneously
      SetPosition(0, elev, arm, intake)  // retract back to stored
    ),

    WaitDrive(4.00_m, 1.0_mps, drive)
    
  );
}