#include "commands/HighDock.h"

HighDock::HighDock(DriveSubsystem *drive, ElevatorSubsystem *elev, ArmSubsystem *arm, IntakeSubsystem *intake) {
  AddCommands(
    SetIntakePower(0.12, intake),       // set intake to holding power

    frc2::ParallelCommandGroup(   // perform simultaneously
      WaitDrive(0.5_m, 0.8_mps, drive),   // drive back from link station

      SetPosition(3, elev, arm, intake)  // extent to mid
    ),

    // TrajectoryRelative({frc::Pose2d{0.0_m, 0.0_m, 0.0_deg}, frc::Pose2d{0.5_m, 0.0_m, 0.0_deg}}, 
    // {AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}, drive),   test line for Trajectory code

    WaitDrive(0.5_m, -0.8_mps, drive),  // drive to link station
    
    // SetPosition(3, elev, arm, intake),  // extent to high
        
    SetIntakePower(-1.0, intake),       // intake full reverse to spit out cone
    
    frc2::WaitCommand(0.5_s),           // wait until cone is out
    
    SetIntakePower(0.0, intake),        // turn off intake

    WaitDrive(0.5_m, 0.5_mps, drive),   // drive back from link station
    
    SetPosition(2, elev, arm, intake),  // retract elevator to mid    
    
    frc2::ParallelCommandGroup(   // perform simultaneously
      SetPosition(0, elev, arm, intake)  // retract back to stored
    ),

    frc2::FunctionalCommand(            // drive until robot hits charge station
      [=] { ; },
      [=] { drive->Drive(2.0_mps, 0_mps, 0_deg_per_s, false); },
      [=] (bool interrupted) { ; },
      [=] { return drive->GetPitch() > 6.0; },
      {drive}),

    frc2::FunctionalCommand(            // drive until robot tips down and then slow down
      [=] { ; },
      [=] { drive->Drive(1.3_mps, 0_mps, 0_deg_per_s, false); },
      [=] (bool interrupted) { drive->Drive(0.4_mps, 0_mps, 0_deg_per_s, false); },
      [=] { return drive->GetPitch() < 2.0; },
      {drive}),
    
    frc2::WaitCommand(1.9_s),         // drive for another extra bit to make sure we break the auto line
    
    frc2::InstantCommand(             // stop the drive
      [=]() { 
        drive->Drive(0.0_mps, 0_mps, 0_deg_per_s, false);
      }, {drive}),

    frc2::FunctionalCommand(            // center the robot
      [=] { ; },
      [=] {
        double delta = 180.0 - (double)drive->GetPose().Rotation().Degrees(); 
        drive->Drive(0.0_mps, 0.0_mps, units::degrees_per_second_t{20.0 + delta * 0.2}, false); 
        },
      [=] (bool interrupted) { drive->Drive(0.0_mps, 0_mps, 0_deg_per_s, false); },
      [=] { return abs(180.0 - (double)drive->GetPose().Rotation().Degrees()) < 10.0; },
      {drive}),
    
    GyroDock(1.5, drive)              // dock on charge station
  );
}