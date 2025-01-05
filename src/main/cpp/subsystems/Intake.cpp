// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Intake.h"
#include "Constants.h"

using namespace IntakeConstants;

/*
Todo: will need to update the type of motor used on the intake once it's known
Todo: Will need to make methods that make sense for what the intake will do
- the intake is planned to be a conveyor so will need to turn one direction at a specific speed
- I'm thinking it should be able to be reversed if we need to 
-it'd be nice to know if we have a note loaded 
*/
Intake::Intake() {
  // Implementation of subsystem constructor goes here.
  //frc::PWMSparkMax m_conveyorMotor(kconveyorMotorPort);
  
}

frc2::CommandPtr Intake::RunIntake() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([ this ] { 
    m_conveyorMotor.Set(kIntakeSpeed);
    frc::SmartDashboard::PutString("Intake","Forward"); 
    });
  //RunOnce creates a command that calls a lambda once, and then finishes.
}

frc2::CommandPtr Intake::ReverseIntake() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([ this ] { 
    m_conveyorMotor.Set(-kIntakeSpeed);
    frc::SmartDashboard::PutString("Intake","Reverse"); 
    });
}

frc2::CommandPtr Intake::Stop(){
return RunOnce([ this ] { 
  m_conveyorMotor.Set(0);
    });
}

bool isLoaded() {
  // Query some boolean state, such as a digital limit switch
  //The shooter will also be able to query this state of this limit switch 
  //If we are loaded, stop trying to load.
  return false;
}

void Intake::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void Intake::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
