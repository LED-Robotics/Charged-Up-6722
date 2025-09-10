#include <units/angle.h>
#include <units/time.h>
#include <units/current.h>

namespace WristConstants {
    // ports
    constexpr int kWristPort = 17;

    constexpr int kEncoderPort = 12;

    constexpr units::angle::turn_t kEncoderOffset =  -0.248779296875_tr;

    constexpr units::time::second_t kRampSeconds = 0.0_s;

    constexpr int kWristDefaultPower = 1.0;
    // Wrist states
    enum WristStates {
      kWristOff,
      kWristPowerMode,
      kWristAngleMode
    };
    // wrist min/max positions
    constexpr units::angle::degree_t kWristDegreeMin{-270.0_deg};   // encoder Turns at the Floor's minimum usable position
    constexpr units::angle::degree_t kWristDegreeMax{180.0_deg}; // meters the Floor is capable of moving 
    // for arm feed forward trig
    // arm position constants
    constexpr double kWristStartOffset = 0.0;
    constexpr units::angle::degree_t kWristStartAngle{-31.0_deg};
    // position deadzone
    constexpr units::angle::degree_t kWristAngleDeadzone{180.0_deg};
    constexpr units::current::ampere_t kCurrentLimit = 30.0_A;

    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.352;
    // TalonFX config
    constexpr double kPWrist = 1.0;
    constexpr double kDWrist = 0.1;
    constexpr double kWristRotorToGearbox = 136.0 / 5.0; //Will change
    constexpr double kWristGearboxToMechanism = 68.0 / 18.0; //Will change
    // constexpr double kTurnsPerDegree = (kWristRotorToGearbox * kWristGearboxToMechanism) / 360.0;
    constexpr double kTurnsPerDegree = (1.0 / 360.0) * 23.64;
}
