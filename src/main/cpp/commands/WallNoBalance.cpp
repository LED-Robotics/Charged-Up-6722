#include "commands/WallNoBalance.h"

WallNoBalance::WallNoBalance(DriveSubsystem *drive, ElevatorSubsystem *elev, ArmSubsystem *arm, IntakeSubsystem *intake) {
  SetName("WallNoBalance");
  AddCommands(

    frc2::InstantCommand(             // stop the drive
      [=]() { 
        arm->SetCubeMode();
    }, {arm}),
    
    frc2::InstantCommand(             // stop the drive
      [=]() { 
        drive->ResetOdometry({0.0_m, 0.0_m, {180.0_deg}});
    }, {drive}),

    frc2::InstantCommand(             // stop the drive
      [=]() { 
        drive->ResetOdometry({0.0_m, 0.0_m, {180.0_deg}});
    }, {drive}),

    frc2::InstantCommand(             // stop the drive
      [=]() { 
        drive->ResetOdometry({0.0_m, 0.0_m, {180.0_deg}});
    }, {drive}),

    SetIntakePower(0.12, intake),       // set intake to holding power

    SetPosition(3, elev, arm, intake),  // extent to mid
        
    SetIntakePower(-1.0, intake),       // intake full reverse to spit out cone
    
    frc2::WaitCommand(0.3_s),           // wait until cone is out
    
    SetIntakePower(0.0, intake),        // turn off intake
    
    frc2::ParallelCommandGroup(
      SetPosition(1, elev, arm, intake),  // retract elevator to ground
      ToPoint({3.25_m, 0.5_m, {180.0_deg}}, drive)
    ),    
        
    SetIntakePower(1.0, intake),       // intake full speed to pick up cube

    ToPoint({3.9_m, 1.7_m, {-45.0_deg}}, drive),
    
    ToPoint({6.6_m, 0.50_m, {-45.0_deg}}, drive),
    
    SetIntakePower(0.12, intake),       // intake holding

    ToPoint({6.15_m, 0.585_m, {180.0_deg}}, drive),
    
    SetPosition(2, elev, arm, intake),  // cube mid

    // ToPoint({0.7_m, 0.14_m, {180.0_deg}}, drive),
    // ToPoint({3.25_m, 0.5_m, {180.0_deg}}, drive),
    
    ToPoint({1.0_m, -0.3_m, {180.0_deg}}, drive),

    SetIntakePower(-1.0, intake),       // intake full reverse to spit out cone
    
    frc2::WaitCommand(0.3_s),           // wait until cone is out

    SetIntakePower(0.0, intake),       // intake full reverse to spit out cone

    frc2::ParallelCommandGroup(
      SetPosition(0, elev, arm, intake),  // arm stored
      ToPoint({3.25_m, 0.5_m, {0.0_deg}}, drive)
      // ToPoint({1.15_m, -1.6_m, {180.0_deg}}, drive)
    )
  );
}