
#include <frc2/command/SubsystemBase.h>
#include <frc/digitalOutput.h>


class LEDS : public frc2::SubsystemBase {
    public:
     LEDS();

    void TeleopPeriodic();

    void TurnOnLED(bool value);
    void TurnOffLED();

    bool areTheyOn();


private:
    frc::DigitalOutput m_led{3}; // PWM pin 1
    bool areTheyOnBro;
};