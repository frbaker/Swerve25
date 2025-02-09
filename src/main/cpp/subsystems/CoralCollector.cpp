// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/CoralCollector.h"
#include "Constants.h"

using namespace CoralCollectorConstants;

CoralCollector::CoralCollector() {
  // Implementation of subsystem constructor goes here.
  //frc::PWMSparkMax m_collector(kconveyorMotorPort);
  SparkMaxConfig defaultConfig;
  SparkMaxConfig smtest2Config;
  smtest2Config.Apply(defaultConfig); //.Follow(m_coralCollectorLeft);
  smtest2Config.Inverted(true);

  m_coralCollectorLeft.Configure(defaultConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
  m_coralCollectorRight.Configure(smtest2Config, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);

}

void CoralCollector::RunCoralCollector() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
    m_coralCollectorLeft.Set(kCoralCollectorSpeed);
    m_coralCollectorRight.Set(kCoralCollectorSpeed);
    frc::SmartDashboard::PutString("CoralCollector","Forward");
  //RunOnce creates a command that calls a lambda once, and then finishes.
}

frc2::CommandPtr CoralCollector::RunCoralCollectorAuto() {
  // Inline construction of command goes here.
  return RunOnce([this] {
    m_coralCollectorLeft.Set(kCoralCollectorSpeed);
    m_coralCollectorRight.Set(kCoralCollectorSpeed);
    });
   
   // frc::SmartDashboard::PutString("CoralCollector","Forward");
  //RunOnce creates a command that calls a lambda once, and then finishes.
}

void CoralCollector::ReverseCoralCollector() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  
    m_coralCollectorLeft.Set(-kCoralCollectorSpeed);
    m_coralCollectorRight.Set(-kCoralCollectorSpeed);
    frc::SmartDashboard::PutString("CoralCollector","Reverse"); 
}

frc2::CommandPtr CoralCollector::ReverseCoralCollectorAuto() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
    return RunOnce([this] {
    m_coralCollectorLeft.Set(-kCoralCollectorSpeed);
    m_coralCollectorRight.Set(-kCoralCollectorSpeed);
    });
   // frc::SmartDashboard::PutString("CoralCollector","Reverse"); 
}

void CoralCollector::Stop(){
  m_coralCollectorLeft.Set(0.0);
  m_coralCollectorRight.Set(0.0);
}

frc2::CommandPtr CoralCollector::StopAuto(){
  return RunOnce([this] {
    m_coralCollectorLeft.Set(0.0);
    m_coralCollectorRight.Set(0.0);
  });
}

void CoralCollector::RunCoralCollectorSlower() {
  m_coralCollectorLeft.Set(kCoralCollectorSpeedSlower);
    m_coralCollectorRight.Set(kCoralCollectorSpeedSlower);
}

void CoralCollector::ReverseCoralCollectorSlower() {
  m_coralCollectorLeft.Set(-kCoralCollectorSpeedSlower);
    m_coralCollectorRight.Set(-kCoralCollectorSpeedSlower);
}

bool isLoaded() {
  // Query some boolean state, such as a digital limit switch
  //The shooter will also be able to query this state of this limit switch 
  //If we are loaded, stop trying to load.
  return false;
}

void CoralCollector::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void CoralCollector::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
