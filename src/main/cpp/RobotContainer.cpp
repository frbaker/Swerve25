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

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  aprilTag.addAprilTagData(1, 122_cm, "source", "blue"); //on the right side of source
  aprilTag.addAprilTagData(2, 122_cm, "source", "blue"); //on the left side of source
  aprilTag.addAprilTagData(3, 132_cm, "speaker", "red"); //shifted 43_cm toward drivers station on speaker
  aprilTag.addAprilTagData(4, 132_cm, "speaker", "red"); //vertically centered on speaker
  aprilTag.addAprilTagData(5, 122_cm, "amp", "red");     //vertically centered on amp
  aprilTag.addAprilTagData(6, 122_cm, "amp", "blue");    //vertically centered on amp
  aprilTag.addAprilTagData(7, 132_cm, "speaker", "blue"); //vertically centered on speaker
  aprilTag.addAprilTagData(8, 132_cm, "speaker", "blue"); //shifted 43_cm toward drivers station on speaker
  aprilTag.addAprilTagData(9, 122_cm, "source", "red"); //on the right side of source
  aprilTag.addAprilTagData(10, 122_cm, "source", "red"); //on the left side of source
  aprilTag.addAprilTagData(11, 121_cm, "stage", "red");
  aprilTag.addAprilTagData(12, 121_cm, "stage", "red");
  aprilTag.addAprilTagData(13, 121_cm, "stage", "red");
  aprilTag.addAprilTagData(14, 121_cm, "stage", "blue");
  aprilTag.addAprilTagData(15, 121_cm, "stage", "blue");
  aprilTag.addAprilTagData(16, 121_cm, "stage", "blue");



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
            true, true);
      },
      {&m_drive}));
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
        //get the camera target info 
//const std::string alliance = frc::DriverStation::GetAlliance();
    if (auto ally = frc::DriverStation::GetAlliance()) {
        if (ally.value() == frc::DriverStation::Alliance::kRed) {
            //we red
            frc::SmartDashboard::PutString("Our Alliance", "RED");
        }
        if (ally.value() == frc::DriverStation::Alliance::kBlue) {
            //we blue
            frc::SmartDashboard::GetNumber("ChooseRoutine", 1);
        }
    }
     


        photon::PhotonPipelineResult result = camera.GetLatestResult();
        result.HasTargets() ? frc::SmartDashboard::PutString("has le target", "true"): frc::SmartDashboard::PutString("has le target", "false");

        if (result.HasTargets()) {
            double Yehaw = result.GetBestTarget().GetPitch();
            int targetID = result.GetBestTarget().GetFiducialId();
            frc::SmartDashboard::PutNumber("Yehaw", Yehaw);
            frc::SmartDashboard::PutNumber("targetID", targetID);
            

            units::length::meter_t targetDataHeight = aprilTag.returnAprilTagDataHeight(targetID);
            const std::string targetDataType = aprilTag.returnAprilTagDataTargetType(targetID);
            const std::string redOrBlue = aprilTag.returnAprilTagDataTargetAlliance(targetID);
            

            

            //Target has to match the alliance we are currently on ie red/blue
            //If we are loaded we only care about speaker targets
            //If we are NOT loaded we only care about source targets

            if (targetID == 23) {

                units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
                CAMERA_HEIGHT, targetDataHeight, CAMERA_PITCH,
                units::degree_t{result.GetBestTarget().GetPitch()});
                m_drive.PhotonDrive(targetID, Yehaw, range); 
                
            }
            
            else {
            //If we don't care about this target - leave it in controll of the driver
            m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true, true);
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
            true, true);
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