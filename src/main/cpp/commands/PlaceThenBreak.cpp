#include "commands/PlaceThenBreak.h"

PlaceThenBreak::PlaceThenBreak(DriveSubsystem *drive, ElevatorSubsystem *elev, ArmSubsystem *arm, IntakeSubsystem *intake) {
  SetName("Place Then Break");
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

    SetPosition(3, elev, arm, intake),  // extend to high
          
    SetIntakePower(-1.0, intake),       // intake full reverse to spit out cube
    
    frc2::WaitCommand(0.6_s),           // wait until cube is out
    
    SetIntakePower(0.0, intake),        // turn off intake

    frc2::ParallelCommandGroup(   // perform simultaneously
      ToPoint({1.0_m, 0.0_m, {180.0_deg}}, drive),   // drive back from station
      SetPosition(0, elev, arm, intake)  // retract back to stored
    ),  

    ToPoint({5.0_m, 0.0_m, {180.0_deg}}, drive)  // get mobility bonus
    
  );
}