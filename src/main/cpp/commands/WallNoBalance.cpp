#include "commands/WallNoBalance.h"

WallNoBalance::WallNoBalance(DriveSubsystem *drive, ElevatorSubsystem *elev, ArmSubsystem *arm, IntakeSubsystem *intake) {
  AddCommands(
    SetIntakePower(0.12, intake),       // set intake to holding power

    // frc2::ParallelCommandGroup(   // perform simultaneously
    //   WaitDrive(0.5_m, 0.8_mps, drive),   // drive back from link station

    //   SetPosition(2, elev, arm, intake)  // extent to mid
    // ),
    // TrajectoryRelative({0.0_m, 0.0_m, 0.0_deg}, {}, {0.25_m, 0.0_m, 0.0_deg}, {AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}, drive),
    // WaitDrive(0.25_m, 0.6_mps, drive),   // drive back from link station
    SetPosition(3, elev, arm, intake),  // extent to mid

    // TrajectoryRelative({frc::Pose2d{0.0_m, 0.0_m, 0.0_deg}, frc::Pose2d{0.5_m, 0.0_m, 0.0_deg}}, 
    // {AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}, drive),   test line for Trajectory code

    // WaitDrive(0.5_m, -0.8_mps, drive),  // drive to link station
    
    // SetPosition(3, elev, arm, intake),  // extent to high
        
    SetIntakePower(-1.0, intake),       // intake full reverse to spit out cone
    
    frc2::WaitCommand(0.5_s),           // wait until cone is out
    
    SetIntakePower(0.0, intake),        // turn off intake
    
    SetPosition(2, elev, arm, intake),  // retract elevator to mid

    WaitDrive(0.2_m, 0.8_mps, drive),   // drive back from link station
    
    frc2::ParallelCommandGroup(   // perform simultaneously
      WaitDrive(0.2_m, 0.8_mps, drive),   // drive back from link station
      SetPosition(0, elev, arm, intake)  // retract back to stored
    ),

    TurnTo(3.0, drive),

    SetPosition(1, elev, arm, intake),  // retract back to stored

    SetIntakePower(1.0, intake),       // intake full reverse to spit out cone

    TrajectoryRelative({0.0_m, 0.0_m, 0_deg}, {}, {5.5_m, 0.0_m, 0_deg}, {AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}, drive),

    SetIntakePower(0.12, intake),

    SetPosition(0, elev, arm, intake),

    TurnTo(180.0, drive),

    TrajectoryRelative({0.0_m, 0.0_m, 0_deg}, {}, {-4.3_m, -0.15_m, 0_deg}, {AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}, drive)

  );
}