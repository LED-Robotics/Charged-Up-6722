

namespace IntakeConstants {
    // Ports
    constexpr int kIntakePort = 19;

    constexpr double kHoldingPower = -0.05;

    // States
    enum IntakeStates {
      kOff,
      kPowerMode
    };

    // Default power in power mode
    constexpr double kDefaultPower = 1.0;
}
