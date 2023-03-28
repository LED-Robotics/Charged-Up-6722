#include "commands/TrajectoryAbsolute.h"

TrajectoryAbsolute::TrajectoryAbsolute(const std::vector<Pose2d>& waypoints, const TrajectoryConfig& config, DriveSubsystem* driveRef) {
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    // waypoints
    waypoints,
    // Pass the config
    config);

  frc::ProfiledPIDController<units::radians> thetaController{
    AutoConstants::kPThetaController, 0, 0,
    AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> command(
    trajectory, [=]() { return driveRef->GetPose(); },

    driveRef->kDriveKinematics,

    frc2::PIDController{AutoConstants::kPXController, 0, 0},
    frc2::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

    [=](auto moduleStates) { driveRef->SetModuleStates(moduleStates); },

    {driveRef});

  AddCommands( 
    command,

    frc2::InstantCommand(             // stop the drive
      [=]() { 
        driveRef->Drive(0.0_mps, 0_mps, 0_deg_per_s, false);
      }, {driveRef})
  );
}