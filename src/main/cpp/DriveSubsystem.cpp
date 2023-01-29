// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DriveSubsystem.h"

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative)
{
  if (m_bOverrideXboxInput == false)
  {
    // auto states = m_kinematics.ToSwerveModuleStates(fieldRelative 
    //   ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
    //   : frc::ChassisSpeeds{xSpeed, ySpeed, rot});
    auto states = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds{xSpeed, ySpeed, rot});

    // Renormalizes the wheel speeds if any individual speed is above the specified maximum
    m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

    auto [fl, fr, bl, br] = states;

    m_frontLeft.SetDesiredState(fl);
    m_frontRight.SetDesiredState(fr);
    m_backLeft.SetDesiredState(bl);
    m_backRight.SetDesiredState(br);
  }
}

void DriveSubsystem::Periodic()
{
  //UpdateOdometry();
  m_frontLeft.Periodic();
  m_frontRight.Periodic();
  m_backLeft.Periodic();
  m_backRight.Periodic();
}

void DriveSubsystem::ResyncAbsRelEnc()
{
  m_frontLeft.ResyncAbsRelEnc();
  m_frontRight.ResyncAbsRelEnc();
  m_backLeft.ResyncAbsRelEnc();
  m_backRight.ResyncAbsRelEnc();
}

void DriveSubsystem::UpdateOdometry()
{
  m_odometry.Update(m_gyro.GetRotation2d(),
                   {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_backLeft.GetPosition(),  m_backRight.GetPosition()});
}

void DriveSubsystem::WheelsForward()
{
  frc::SwerveModuleState sms;
  sms.angle = frc::Rotation2d{0.0_deg};
  sms.speed = 0.0_mps;
  SetAllDesiredState(sms);
}

void DriveSubsystem::WheelsLeft()
{
  frc::SwerveModuleState sms;
  sms.angle = frc::Rotation2d{90.0_deg};
  sms.speed = 0.0_mps;
  SetAllDesiredState(sms);
}

void DriveSubsystem::WheelsBackward()
{
  frc::SwerveModuleState sms;
  sms.angle = frc::Rotation2d{150.0_deg};
  sms.speed = 0.0_mps;
  SetAllDesiredState(sms);
}

void DriveSubsystem::WheelsRight()
{
  frc::SwerveModuleState sms;
  sms.angle = frc::Rotation2d{-45.0_deg};
  sms.speed = 0.0_mps;
  SetAllDesiredState(sms);
}

void DriveSubsystem::SetAllDesiredState(const frc::SwerveModuleState& sms)
{
  m_frontLeft.SetDesiredState(sms);
  m_frontRight.SetDesiredState(sms);
  m_backLeft.SetDesiredState(sms);
  m_backRight.SetDesiredState(sms);
}