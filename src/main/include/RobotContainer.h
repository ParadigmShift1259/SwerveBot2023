// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>

#include <DriveSubsystem.h>

class RobotContainer
{
public:
  RobotContainer();
  
  frc2::CommandPtr GetAutonomousCommand();
  void Periodic() { m_drive.Periodic(); }

private:
  void SetDefaultCommands();
  void ConfigureBindings();
  DriveSubsystem m_drive;

  frc::XboxController m_primaryController{0};
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  bool m_fieldRelative = false;//true; TODO

  frc2::InstantCommand m_wheelsForward{[this] { m_drive.WheelsForward(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsLeft{[this] { m_drive.WheelsLeft(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsBackward{[this] { m_drive.WheelsBackward(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsRight{[this] { m_drive.WheelsRight(); }, {&m_drive} };

  frc2::InstantCommand m_OverrideOn{[this] { m_drive.SetOverrideXboxInput(true); }, {&m_drive} };
  frc2::InstantCommand m_OverrideOff{[this] { m_drive.SetOverrideXboxInput(false); }, {&m_drive} };

  frc2::InstantCommand m_resyncAbsRelEnc{[this] { m_drive.ResyncAbsRelEnc(); }, {&m_drive} };
};
