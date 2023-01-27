// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>

#include <DriveSubsystem.h>

class RobotContainer
{
public:
  RobotContainer();
  
  frc2::CommandPtr GetAutonomousCommand();

private:
  void SetDefaultCommands();
  void ConfigureBindings();
  DriveSubsystem m_drive;

  frc::XboxController m_primaryController{0};
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  bool m_fieldRelative = true;
};
