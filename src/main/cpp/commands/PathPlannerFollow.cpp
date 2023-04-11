#include "commands/PathPlannerFollow.h"

PathPlannerFollow::PathPlannerFollow(int target, DriveSubsystem *driveRef) {
  AddRequirements(driveRef);
  auto current = driveRef->GetPose();
  units::meter_t targetY = 5.0_m;
  switch(target) {
    case 0:
      targetY = 5.0_m;
      break;
    case 1:
      targetY = 4.42_m;
      break;
    case 2:
      targetY = 3.88_m;
      break;
    case 3:
      targetY = 3.30_m;
      break;
    case 4:
      targetY = 2.75_m;
      break;
    case 5:
      targetY = 2.20_m;
      break;
    case 6:
      targetY = 1.60_m;
      break;
    case 7:
      targetY = 1.05_m;
      break;
    case 8:
      targetY = 0.5_m;
      break;
  }
  double heading = -90.0;
  if((double)targetY > (double)current.Y()) heading *= -1.0;
  SmartDashboard::PutNumber("targetNumber", target);
  SmartDashboard::PutNumber("targetY", (double)targetY);
  PathPlannerTrajectory traj = PathPlanner::generatePath(
    PathConstraints(2.5_mps, 2_mps_sq), 
    PathPoint(frc::Translation2d(current.X(), current.Y()), frc::Rotation2d(units::degree_t{heading}), current.Rotation()), // position, heading(direction of travel), holonomic rotation
    PathPoint(frc::Translation2d(2.00_m, targetY), frc::Rotation2d(units::degree_t{heading}), current.Rotation()) // position, heading(direction of travel) holonomic rotation
  );

  PPSwerveControllerCommand drive(
    traj,
    [=]() { return driveRef->GetPose(); },
    frc2::PIDController{0.0, 0.0, 0.0},
    frc2::PIDController{0.0, 0.0, 0.0},
    frc2::PIDController{0.0, 0.0, 0.0},
    [=](auto speeds) { driveRef->Drive(speeds.vx, speeds.vy, speeds.omega, false); },
    {driveRef},
    false
  );

  AddCommands(drive);
}