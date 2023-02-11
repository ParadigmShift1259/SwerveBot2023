// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/button/JoystickButton.h>

RobotContainer::RobotContainer() : m_drive()
{
  SetDefaultCommands();
  ConfigureBindings();

  frc::SmartDashboard::PutNumber("PitchFactor", m_pitchFactor);
  frc::SmartDashboard::PutNumber("MaxAutoBalanceSpeed", m_maxAutoBalanceSpeed);
}

//frc2::CommandPtr RobotContainer::GetAutonomousCommand()
frc2::Command* RobotContainer::GetAutonomousCommand()
{
  std::vector<frc::Pose2d> waypoints
  {
    frc::Pose2d(3.95_m, 4.17_m, 0_deg),
    frc::Pose2d(3.95_m, 2.17_m, 0_deg),
    frc::Pose2d(2.95_m, 2.17_m, 0_deg),
    frc::Pose2d(2.95_m, 4.17_m, 0_deg),
    frc::Pose2d(3.95_m, 4.17_m, 0_deg)
  };

  auto config = frc::TrajectoryConfig(units::velocity::meters_per_second_t{3.5}, units::meters_per_second_squared_t{1.0});
  config.SetKinematics(m_drive.m_kinematics);
  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(waypoints, config);
  
  //return (frc2::CommandPtr)GetSwerveCommandPath(trajectory);
  return GetSwerveCommandPath(trajectory);
  // return frc2::cmd::Print("No autonomous command configured");
}

void RobotContainer::Periodic() {
  m_drive.Periodic();

  m_pitchFactor = frc::SmartDashboard::GetNumber("PitchFactor", m_pitchFactor);
  m_maxAutoBalanceSpeed = frc::SmartDashboard::GetNumber("MaxAutoBalanceSpeed", m_maxAutoBalanceSpeed);
}

void RobotContainer::SetDefaultCommands()
{
  m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] 
    {
      //const double kDeadband = 0.02;
      const double kDeadband = 0.1;
      const auto xInput = frc::ApplyDeadband(m_primaryController.GetLeftY(), kDeadband);
      const auto yInput = frc::ApplyDeadband(m_primaryController.GetLeftX(), kDeadband);
      const auto rotInput = frc::ApplyDeadband(m_primaryController.GetRightX(), kDeadband);

      const auto xSpeed = m_xspeedLimiter.Calculate(xInput) * kMaxSpeed;
      const auto ySpeed = m_yspeedLimiter.Calculate(yInput) * kMaxSpeed;
      const auto rot = m_rotLimiter.Calculate(rotInput) * kMaxAngularSpeed;
      
      m_drive.Drive(xSpeed, ySpeed, rot, m_fieldRelative);
    },
    {&m_drive}
  ));
}

void RobotContainer::ConfigureBindings()
{
    using namespace frc;
    using namespace frc2;
    using xbox = frc::XboxController::Button;

    auto& primary = m_primaryController;

    // Primary
    // Keep the bindings in this order
    // A, B, X, Y, Left Bumper, Right Bumper, Back, Start
    JoystickButton(&primary, xbox::kA).WhileTrue(&m_wheelsBackward);
    // JoystickButton(&primary, xbox::kB).WhileTrue(&m_wheelsLeft);
    JoystickButton(&primary, xbox::kX).WhileTrue(&m_wheelsRight);
    JoystickButton(&primary, xbox::kY).WhileTrue(&m_wheelsForward);

    JoystickButton(&primary, xbox::kY).WhileTrue(GetParkAndBalanceCommand());
    JoystickButton(&primary, xbox::kB).OnTrue(GetParkCommand());

    JoystickButton(&primary, xbox::kStart).WhileTrue(&m_OverrideOn);
    JoystickButton(&primary, xbox::kBack).WhileTrue(&m_OverrideOff);

    //JoystickButton(&primary, xbox::kRightBumper).WhileTrue(&m_resyncAbsRelEnc);
    // Triggers field relative driving
    // TODO If we set field relative as default, we also need to swap the 
    //      button bindings here (while button is true (pressed) it should clear field relative (be robo relative))
    JoystickButton(&primary, xbox::kLeftBumper).WhileTrue(&m_setFieldRelative);
    JoystickButton(&primary, xbox::kLeftBumper).WhileFalse(&m_clearFieldRelative);

}

frc2::SequentialCommandGroup* RobotContainer::GetParkCommand()
{
    return new frc2::SequentialCommandGroup
    (
        frc2::ParallelDeadlineGroup
        (
              frc2::WaitUntilCommand([this]() { return m_drive.GetPitch() < -7.0; })
            , frc2::RunCommand([this]() { m_drive.Drive(-1.00_mps, 0.0_mps, 0.0_rad_per_s, false); }, {&m_drive})
        )
        , frc2::ParallelDeadlineGroup
        (
            frc2::WaitCommand(1.600_s)
          , frc2::RunCommand([this]() { m_drive.Drive(-1.00_mps, 0.0_mps, 0.0_rad_per_s, false); }, {&m_drive})
        )
        , frc2::InstantCommand([this]() { m_drive.Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false); }, {&m_drive})
    );
}

frc2::ConditionalCommand* RobotContainer::GetParkAndBalanceCommand()
{
    return new frc2::ConditionalCommand
    (
        frc2::RunCommand([this]() { m_drive.Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false); }, {&m_drive})    // Cmd if true
      , frc2::RunCommand([this]()                                                                            // Cmd if false
        { 
          double driveSpeed = std::clamp(m_pitchFactor * m_drive.GetPitch(), -m_maxAutoBalanceSpeed, m_maxAutoBalanceSpeed);
          m_drive.Drive(units::velocity::meters_per_second_t(driveSpeed), 0.0_mps, 0.0_rad_per_s, false); 
        }
        , {&m_drive})
      , [this]() { return m_drive.GetPitch() > -1.0 && m_drive.GetPitch() < 1.0; }    // Condition
    );
}

const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints{kMaxAngularSpeed, kMaxAngularAcceleration};


frc2::SwerveControllerCommand<4>* RobotContainer::GetSwerveCommandPath(frc::Trajectory trajectory)
{
    frc::ProfiledPIDController<units::radians> thetaController{2.0, 0, 1.0, kThetaControllerConstraints};

    thetaController.EnableContinuousInput(units::radian_t(-std::numbers::pi), units::radian_t(std::numbers::pi));

    frc2::SwerveControllerCommand<4>* swerveControllerCommand = new frc2::SwerveControllerCommand<4>(
        trajectory,                                                             // frc::Trajectory
        [this]() { return m_drive.GetPose(); },                                 // std::function<frc::Pose2d()>
        m_drive.m_kinematics,                                               // frc::SwerveDriveKinematics<NumModules>
        frc2::PIDController(1.0, 0, 0.0),                // frc2::PIDController
        frc2::PIDController(1.0, 0, 0.0),                // frc2::PIDController
        thetaController,                                                        // frc::ProfiledPIDController<units::radians>
        [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
        {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
    );

    return swerveControllerCommand;
}
