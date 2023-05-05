#include "commands/HighDock.h"

HighDock::HighDock(DriveSubsystem *drive, ElevatorSubsystem *elev, ArmSubsystem *arm, IntakeSubsystem *intake) {
  SetName("High Dock");
  AddCommands(
    frc2::InstantCommand(             // set to cube placement mode
      [=]() { 
        arm->SetCubeMode();
    }, {arm}),

    frc2::InstantCommand(             // reset odometry. This doesn't always work do it three times
      [=]() { 
        drive->ResetOdometry({0.0_m, 0.0_m, {180.0_deg}});
    }, {drive}),

    frc2::InstantCommand(             // reset odometry.
      [=]() { 
        drive->ResetOdometry({0.0_m, 0.0_m, {180.0_deg}});
    }, {drive}),

    frc2::InstantCommand(             // reset odometry.
      [=]() { 
        drive->ResetOdometry({0.0_m, 0.0_m, {180.0_deg}});
    }, {drive}),

    SetIntakePower(0.12, intake),       // set intake to holding power

    SetPosition(3, elev, arm, intake),  // extent to mid
        
    SetIntakePower(-1.0, intake),       // intake full reverse to spit out cube
    
    frc2::WaitCommand(0.5_s),           // wait until cube is out
    
    SetIntakePower(0.0, intake),        // turn off intake

    
    ToPoint({1.0_m, 0.0_m, {180.0_deg}}, drive),   // drive back from station

    SetPosition(0, elev, arm, intake),  // retract back to stored

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

    ToPoint({5.1_m, 0.0_m, {180.0_deg}}, drive),   // drive back from link station

    frc2::WaitCommand(0.25_s),         // wait for charge station to stop teetering
    
    frc2::InstantCommand(             // stop the drive
      [=]() { 
        drive->Drive(0.0_mps, 0_mps, 0_deg_per_s, false);
      }, {drive}),
    
    GyroDock(-1.8, drive)              // dock on charge station
  );
}