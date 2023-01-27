// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

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
      const auto xSpeed = -m_xspeedLimiter.Calculate(frc::ApplyDeadband(m_primaryController.GetLeftY(), 0.02)) * DriveSubsystem::kMaxSpeed;
      const auto ySpeed = -m_yspeedLimiter.Calculate(frc::ApplyDeadband(m_primaryController.GetLeftX(), 0.02)) * DriveSubsystem::kMaxSpeed;
      const auto rot = -m_rotLimiter.Calculate(frc::ApplyDeadband(m_primaryController.GetRightX(), 0.02)) * DriveSubsystem::kMaxAngularSpeed;
      m_drive.Drive(xSpeed, ySpeed, rot, m_fieldRelative);
    },
    {&m_drive}
  ));
}

void RobotContainer::ConfigureBindings()
{

}