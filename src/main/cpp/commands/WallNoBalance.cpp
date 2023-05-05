#include "commands/WallNoBalance.h"

WallNoBalance::WallNoBalance(DriveSubsystem *drive, ElevatorSubsystem *elev, ArmSubsystem *arm, IntakeSubsystem *intake) {
  SetName("WallNoBalance");
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
        
    SetIntakePower(-1.0, intake),       // intake full reverse to spit out cone
    
    frc2::WaitCommand(0.3_s),           // wait until cube is out
    
    SetIntakePower(0.0, intake),        // turn off intake
    
    frc2::ParallelCommandGroup(   // move to ground pos and move away from station
      SetPosition(1, elev, arm, intake),
      ToPoint({3.25_m, 0.5_m, {180.0_deg}}, drive)
    ),    
        
    SetIntakePower(1.0, intake),       // intake full speed to pick up cube

    ToPoint({3.9_m, 1.7_m, {-45.0_deg}}, drive),  // drive to weird angle
    
    ToPoint({6.6_m, 0.50_m, {-45.0_deg}}, drive), // drive into cube at a 45 to make it more likely we pick it up
    
    SetIntakePower(0.12, intake),       // intake holding power

    ToPoint({6.15_m, 0.585_m, {180.0_deg}}, drive),   // turn around
    
    SetPosition(2, elev, arm, intake),  // set position to place cube mid
    
    ToPoint({1.0_m, -0.3_m, {180.0_deg}}, drive),   // move to station

    SetIntakePower(-1.0, intake),       // intake full reverse to spit out cube
    
    frc2::WaitCommand(0.3_s),           // wait until cube is out

    SetIntakePower(0.0, intake),       // intake full reverse to spit out cube

    frc2::ParallelCommandGroup(
      SetPosition(0, elev, arm, intake),  // arm stored
      ToPoint({3.25_m, 0.5_m, {0.0_deg}}, drive)  // end turned around
      // ToPoint({1.15_m, -1.6_m, {180.0_deg}}, drive)  // move to charge station
    )
  );
}