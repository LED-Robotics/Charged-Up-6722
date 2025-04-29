#include <units/length.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/time.h>

namespace TelescopeConstants {
    // ports
    constexpr int kLeftMotorPort = 12;
    constexpr int kRightMotorPort = 13;
    constexpr int kEncoderPort = 14;

    constexpr units::angle::turn_t kEncoderOffset = 0.498291015625_tr;

    constexpr units::time::second_t kRampSeconds = 0.5_s;

    // states
    enum TelescopeStates {
      kOff,
      kPowerMode,
      kPositionMode
    };
    // default power in power mode

    constexpr double kDefaultPower = 0.0;
    constexpr double kStageMultiplier = 0.0;
    // arm position constants
    constexpr units::length::meter_t kStartPosition{0.0_m};
    // constexpr units::length::meter_t kStartPosition{0.13335_m};
    // arm min/max positions
    constexpr units::length::meter_t kTelescopeMeterMin{kStartPosition};   // encoder Turns at the Telescope's minimum usable position
    constexpr units::length::meter_t kTelescopeMeterMax{0.5_m}; // meters the Telescope is capable of moving 
    // for arm feed forward trig
    constexpr double kTurnsPerMeter = 2.46056189903846; // Recalc
    // position deadzone
    constexpr units::length::meter_t kPositionDeadzone{10.0_cm};
    constexpr units::current::ampere_t kCurrentLimit = 30.0_A;
    // TalonFX config
    constexpr double kP = 0.0;
    constexpr double kD = 0.0;
    /*constexpr double kD = 0.0;*/
    constexpr double kG = 0.0;
    /*constexpr double kG = 0.0;*/
    constexpr double kRotorToGearbox = 25.0;
}
