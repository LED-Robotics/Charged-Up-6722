#include <units/angle.h>
#include <units/time.h>
#include <units/current.h>

namespace ArmConstants {
    // ports
    constexpr int kLeftArmPort = 14;
    constexpr int kRightArmPort = 15;

    constexpr units::angle::turn_t kEncoderOffset =  -0.248779296875_tr;

    constexpr units::time::second_t kRampSeconds = 0.25_s;

    constexpr int kArmDefaultPower = 1.0;
    // Arm states
    enum ArmStates {
      kArmOff,
      kArmPowerMode,
      kArmAngleMode
    };
    // wrist min/max positions
    constexpr units::angle::degree_t kArmDegreeMin{-360.0_deg};   // encoder Turns at the Floor's minimum usable position
    constexpr units::angle::degree_t kArmDegreeMax{180.0_deg}; // meters the Floor is capable of moving 
    // for arm feed forward trig
    // arm position constants
    constexpr double kArmStartOffset = 0.0;
    constexpr units::angle::degree_t kArmStartAngle{10.0_deg};
    // position deadzone
    constexpr units::angle::degree_t kArmAngleDeadzone{10.0_deg};
    constexpr units::current::ampere_t kCurrentLimit = 30.0_A;

    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.352;
    // TalonFX config
    constexpr double kPArm = 140.0;
    constexpr double kDArm = 12.0;
    constexpr double kArmRotorToGearbox = 136.0 / 5.0; //Will change
    constexpr double kArmGearboxToMechanism = 68.0 / 18.0; //Will change
    // constexpr double kTurnsPerDegree = (kArmRotorToGearbox * kArmGearboxToMechanism) / 360.0;
    constexpr double kTurnsPerDegree = 1.0 / 360.0;
}
