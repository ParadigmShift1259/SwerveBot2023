// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix.h>

#include "SwerveModule.h"
#include "Gyro.h"

/**
 * Represents a swerve drive style DriveSubsystem.
 */
class DriveSubsystem : public frc2::SubsystemBase
{
public:
  DriveSubsystem() { m_gyro.Reset(); }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);
  void UpdateOdometry();

  static constexpr units::meters_per_second_t kMaxSpeed = 3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{std::numbers::pi};  // 1/2 rotation per second

private:
  static constexpr auto kTrackWidth = 20_in;
  static constexpr auto kWheelBase = 28_in;

  frc::Translation2d m_frontLeftLocation{kWheelBase / 2, kTrackWidth / 2};
  frc::Translation2d m_frontRightLocation{kWheelBase / 2, -kTrackWidth / 2};
  frc::Translation2d m_backLeftLocation{-kWheelBase / 2, kTrackWidth / 2};
  frc::Translation2d m_backRightLocation{-kWheelBase / 2, -kTrackWidth / 2};

  SwerveModule m_frontLeft{1, 2, false, 0.5};//159.8};
  SwerveModule m_frontRight{3, 4, false, 0.633333};//228.9};
  SwerveModule m_backRight{5, 6, false, 0.01};
  SwerveModule m_backLeft{7, 8, true, 0.01};

  Gyro m_gyro;

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, 
      m_backLeftLocation, m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{
      m_kinematics,
      m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()}};
};
