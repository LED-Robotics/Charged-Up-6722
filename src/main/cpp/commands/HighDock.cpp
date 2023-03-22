#include "commands/HighDock.h"

HighDock::HighDock(DriveSubsystem *drive, ElevatorSubsystem *elev, ArmSubsystem *arm, IntakeSubsystem *intake) {
  AddCommands(
    SetIntakePower(0.12, intake),       // set intake to holding power

    frc2::ParallelCommandGroup(   // perform simultaneously
      WaitDrive(0.5_m, 0.7_mps, drive),   // drive back from link station

      SetPosition(2, elev, arm, intake)  // extent to mid
    ),
    // below two lines are non-parallel version
    // WaitDrive(0.7_m, 0.7_mps, drive),   // drive back from link station

    // SetPosition(2, elev, arm, intake),  // extent to mid

    WaitDrive(-0.7_m, 0.7_mps, drive),  // drive to link station
    
    SetPosition(3, elev, arm, intake),  // extent to high
        
    SetIntakePower(-1.0, intake),       // intake full reverse to spit out cone
    
    frc2::WaitCommand(0.5_s),           // wait until cone is out
    
    SetIntakePower(0.0, intake),        // turn off intake
    
    SetPosition(2, elev, arm, intake),  // retract elevator to mid
    
    frc2::ParallelCommandGroup(   // perform simultaneously
      WaitDrive(0.5_m, 0.7_mps, drive),   // drive back from link station

      SetPosition(0, elev, arm, intake)  // retract back to stored
    ),
    // below two lines are non-parallel version
    // WaitDrive(0.7_m, 0.7_mps, drive),   // drive back from link station

    // SetPosition(0, elev, arm, intake),  // retract back to stored

    frc2::FunctionalCommand(            // drive until robot hits charge station
      [=] { ; },
      [=] { drive->Drive(2.5_mps, 0_mps, 0_deg_per_s, false); },
      [=] (bool interrupted) { ; },
      [=] { return drive->GetPitch() > 6.0; },
      {drive}),

    frc2::FunctionalCommand(            // drive until robot tips down and then slow down
      [=] { ; },
      [=] { drive->Drive(2.5_mps, 0_mps, 0_deg_per_s, false); },
      [=] (bool interrupted) { drive->Drive(0.7_mps, 0_mps, 0_deg_per_s, false); },
      [=] { return drive->GetPitch() < 2.0; },
      {drive}),
    
    frc2::WaitCommand(1.5_s),         // drive for another extra bit to make sure we break the auto line
    
    frc2::InstantCommand(             // stop the drive
      [=]() { 
        drive->Drive(0.0_mps, 0_mps, 0_deg_per_s, false);
      }, {drive}),
    
    GyroDock(2.0, drive)              // dock on charge station
  );
}