

#pragma once

#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <frc/PWM.h>

using namespace rev::spark;


class Climber : public frc2::SubsystemBase{
    public:
        Climber();


        void RunClimber(double runSpeed);

        bool MovePigeon();

        void Periodic() override;

        void SimulationPeriodic() override;

        double EncoderValue();


    private:
        SparkMax m_climberMotor{ClimberConstants::kClimberMotorCanId, SparkLowLevel::MotorType::kBrushless};
        frc::PWM m_pigeonServo{0};
        SparkRelativeEncoder m_climberEncoder = m_climberMotor.GetEncoder();

};