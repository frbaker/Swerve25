// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Arm.h"
#include <photon/PhotonUtils.h>
#include "utils/AprilTagData.h"
#include <frc/DriverStation.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>

using namespace pathplanner;
using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Bottom of each tags height - note barge tags are angled down 30 degrees and aproximately centered on the middle cage
  aprilTag.addAprilTagData(1, 135_cm, "CPU", "red"); //Coral pick up red left of drivers stations (13)
  aprilTag.addAprilTagData(2, 135_cm, "CPU", "red"); //Coral pick up red right of drivers stations (12)
  aprilTag.addAprilTagData(3, 117_cm, "processor", "red"); //Processor red alliance (16)
  aprilTag.addAprilTagData(4, 178_cm, "barge", "red"); //blue end of Barge on the red alliance end of field (15)
  aprilTag.addAprilTagData(5, 178_cm, "barge", "red"); //red end of barge on red alliance end of field (14)
  aprilTag.addAprilTagData(6, 17_cm, "reef", "red"); //Reef left of center closer to drivers stations (19)
  aprilTag.addAprilTagData(7, 17_cm, "reef", "red"); //Reef center closer to drivers stations (18)
  aprilTag.addAprilTagData(8, 17_cm, "reef", "red"); //Reef right of center closer to drivers stations (17)
  aprilTag.addAprilTagData(9, 17_cm, "reef", "red"); //Reef right of center closer to barge (as viewed from DS) (22)
  aprilTag.addAprilTagData(10, 17_cm, "reef", "red"); //Reef center closer to Barge (21)
  aprilTag.addAprilTagData(11, 17_cm, "reef", "red"); //Reef left of center closer to barge (as viewed from DS) (20)
  aprilTag.addAprilTagData(12, 135_cm, "CPU", "blue"); //Coral pick up blue right of drivers stations (2)
  aprilTag.addAprilTagData(13, 135_cm, "CPU", "blue"); //Coral pick up blue left of drivers stations (1)
  aprilTag.addAprilTagData(14, 178_cm, "barge", "blue"); //blue end of barge on blue alliance end of field (5) 
  aprilTag.addAprilTagData(15, 178_cm, "barge", "blue"); //red end of barge on blue alliance end of field (4)
  aprilTag.addAprilTagData(16, 117_cm, "processor", "blue"); //Processor blue alliance (3)
  aprilTag.addAprilTagData(17, 17_cm, "reef", "blue"); //Reef right of center closer to drivers stations (8)
  aprilTag.addAprilTagData(18, 17_cm, "reef", "blue"); //Reef center closer to drivrs stations (7)
  aprilTag.addAprilTagData(19, 17_cm, "reef", "blue"); //Reef left of center closer to drivers stations(6)
  aprilTag.addAprilTagData(20, 17_cm, "reef", "blue"); //Reef left of center closer to barge (as viewed from DS) (11)
  aprilTag.addAprilTagData(21, 17_cm, "reef", "blue"); //Reef center closer to barge (10)
  aprilTag.addAprilTagData(22, 17_cm, "reef", "blue"); //Reef right of center closer to barge (as viewed from DS) (9)




  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            false,true);
      },
      {&m_drive}));
}
bool RobotContainer::isValueInArray(int value, int array[], int size) {
    for (int i = 0; i < size; ++i) {
        if (array[i] == value) {
            return true;
        }
    }
    return false;
}
void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kRightBumper)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));


    frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kX)
      .WhileTrue(new frc2::RunCommand([this] { m_intake.RunIntake(); }, {&m_intake}));     

    frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kY)
      .WhileTrue(new frc2::RunCommand([this] { m_intake.ReverseIntake(); }, {&m_intake}));  

    frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kLeftBumper)
      .WhileTrue(new frc2::RunCommand([this] { m_intake.Stop(); }, {&m_intake}));      


    frc2::JoystickButton(&m_driverController,
                    frc::XboxController::Button::kA)
    .OnTrue(new frc2::RunCommand([this] { 
        m_arm.RunArm(); 
    }, {&m_arm}))
    .OnFalse(new frc2::RunCommand([this] { 
        m_arm.Stop(); 
    }, {&m_arm}));

    frc2::JoystickButton(&m_driverController,
                    frc::XboxController::Button::kB)
    .WhileTrue(new frc2::RunCommand([this] { 
          m_arm.Stop();
        }, {&m_arm})); 


//Second Controller
    frc2::JoystickButton(&m_coDriverController,
                       frc::XboxController::Button::kLeftBumper)
      .WhileTrue(new frc2::RunCommand([this] { 
        frc::SmartDashboard::PutString("Running", "PhotonDrive");
        //get the camera target info 
//const std::string alliance = frc::DriverStation::GetAlliance();
            auto ally = frc::DriverStation::GetAlliance();
            if (ally.value() == frc::DriverStation::Alliance::kRed) {
                //we red
                frc::SmartDashboard::PutString("Our Alliance", "RED");
            }
            else if (ally.value() == frc::DriverStation::Alliance::kBlue) {
                //we blue
                frc::SmartDashboard::GetNumber("ChooseRoutine", 1);
                frc::SmartDashboard::PutString("Our Alliance", "BLUE");
            }
            else {
                frc::SmartDashboard::PutString("Our Alliance", "Unknown");
            }
        
     


        std::vector<photon::PhotonPipelineResult> results = camera.GetAllUnreadResults();
        if (!results.empty()) {
            frc::SmartDashboard::PutString("Running", "PhotonDrive hasResults");
            photon::PhotonPipelineResult result = results.back(); //back gets only the most recent
            result.HasTargets() ? frc::SmartDashboard::PutString("has le target", "true"): frc::SmartDashboard::PutString("has le target", "false");
            if (result.HasTargets()) {
                frc::SmartDashboard::PutString("Running", "PhotonDrive HasTargets");
                int reefTags[12] = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
                for (auto target : result.GetTargets()) {
                    double Yehaw = result.GetBestTarget().GetPitch();
                    int targetID = result.GetBestTarget().GetFiducialId();
                    bool found = std::any_of(std::begin(reefTags), std::end(reefTags), [targetID](int x) { return x == targetID; });
                    if (found) {
                        units::length::meter_t targetDataHeight = aprilTag.returnAprilTagDataHeight(targetID);
                        const std::string targetDataType = aprilTag.returnAprilTagDataTargetType(targetID);
                        const std::string redOrBlue = aprilTag.returnAprilTagDataTargetAlliance(targetID);
                        units::degree_t yaw = units::degree_t(target.GetYaw());
                        units::degree_t pitch = units::degree_t(target.GetPitch());
                        units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
                        CAMERA_HEIGHT, targetDataHeight, CAMERA_PITCH,
                        units::degree_t{result.GetBestTarget().GetPitch()});
                        units::meter_t targetDistance = units::meter_t(target.GetPoseAmbiguity() * range); // This might not be the actual calculation, adjust as needed
                        
                        //m_drive.PhotonDrive(targetID, Yehaw, range, yaw, targetDistance);
                    }
                    else {
                        //If we don't care about this target - leave it in control of the driver
                        m_drive.Drive(
                        -units::meters_per_second_t{frc::ApplyDeadband(
                            m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
                        -units::meters_per_second_t{frc::ApplyDeadband(
                            m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
                        -units::radians_per_second_t{frc::ApplyDeadband(
                            m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
                        false, true);
                    }
                }
            }  
            else {
                frc::SmartDashboard::PutString("Running", "PhotonDrive No targtes");
                //If we don't have any vision targets - leave it in controll of the driver
                m_drive.Drive(
                -units::meters_per_second_t{frc::ApplyDeadband(
                    m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
                -units::meters_per_second_t{frc::ApplyDeadband(
                    m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
                -units::radians_per_second_t{frc::ApplyDeadband(
                    m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
                false, true);
            }
        }
         else {
            
             //If we don't have any vision targets - leave it in controll of the driver
            m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            false, true);
        }
      }, {&m_drive}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {

  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
        // positive 2nd number = left, negative = right
        // Pass through these two interior waypoints, making an 's' curve path

      {
        frc::Translation2d{1_m, 0_m},
       frc::Translation2d{2_m, 0_m},},
       
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{2_m, 0_m, 90_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand([this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); },{}),
      frc2::RunCommand([this] { 
        m_arm.RunArm(); 
    }, {&m_arm})
        
  );
}
