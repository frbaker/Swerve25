

#include "subsystems/Climber.h"
#include "rev/SparkMax.h"
#include <rev/config/SparkMaxConfig.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ClimberConstants;

Climber::Climber(){
    SparkMaxConfig climbFollowerConfigObj;
    climbFollowerConfigObj.OpenLoopRampRate(1.75);
    m_climberMotor.Configure(climbFollowerConfigObj, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
}


void Climber::Periodic(){

}

void Climber::SimulationPeriodic(){

}

void Climber::RunClimber(double runSpeed){
    m_climberMotor.Set(runSpeed);
}

void Climber::MovePigeon(){
    frc::SmartDashboard::PutNumber("pigeonServo",m_pigeonServo.GetPosition());
    if(m_pigeonServo.GetPosition() == 0.95){
        m_pigeonServo.SetPosition(0);
    }
    else{
        m_pigeonServo.SetPosition(0.95);
    }
}