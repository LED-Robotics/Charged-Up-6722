#include "commands/PathPlannerFollow.h"

PathPlannerFollow::PathPlannerFollow(DriveSubsystem* driveRef) {


// This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
std::vector<PathPlannerTrajectory> pathGroup = PathPlanner::loadPathGroup("AutoPath", {PathConstraints(4_mps, 3_mps_sq)});

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands/autobuilders.
std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;
// eventMap.emplace("marker1", std::make_shared<frc2::PrintCommand>("Passed Marker 1"));
// eventMap.emplace("intakeDown", std::make_shared<frc2::PrintCommand>("Passed Marker 2"));

// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this could be in RobotContainer along with your subsystems

SwerveAutoBuilder autoBuilder(
    [=]() { return driveRef->GetPose(); }, // Function to supply current robot pose
    [=](auto initPose) { driveRef->ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
    PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    [=](auto speeds) { driveRef->Drive(speeds.vx, speeds.vy, speeds.omega, true); }, // Output function that accepts field relative ChassisSpeeds
    eventMap, // Our event map
    { driveRef }, // Drive requirements, usually just a single drive subsystem
    true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
);

frc2::CommandPtr fullAuto = autoBuilder.fullAuto(pathGroup);

  AddCommands( 

    frc2::InstantCommand(             // stop the drive
      [=]() { 
        driveRef->Drive(0.0_mps, 0_mps, 0_deg_per_s, false);
      }, {driveRef})
  );
}