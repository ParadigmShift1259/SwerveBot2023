// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/MathUtil.h>

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/button/JoystickButton.h>

RobotContainer::RobotContainer() : m_drive()
{
  SetDefaultCommands();
  ConfigureBindings();
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  return frc2::cmd::Print("No autonomous command configured");
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

      const auto xSpeed = m_xspeedLimiter.Calculate(xInput) * DriveSubsystem::kMaxSpeed;
      const auto ySpeed = m_yspeedLimiter.Calculate(yInput) * DriveSubsystem::kMaxSpeed;
      const auto rot = m_rotLimiter.Calculate(rotInput) * DriveSubsystem::kMaxAngularSpeed;
      
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
          double driveSpeed = std::clamp(0.033 * m_drive.GetPitch(), -0.5, 0.5);
          m_drive.Drive(units::velocity::meters_per_second_t(driveSpeed), 0.0_mps, 0.0_rad_per_s, false); 
        }
        , {&m_drive})
      , [this]() { return m_drive.GetPitch() > -1.0 && m_drive.GetPitch() < 1.0; }    // Condition
    );
}
