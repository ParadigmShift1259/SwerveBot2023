// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include <wpi/DataLog.h>

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/DataLogManager.h>
#include <frc/Timer.h>
#include <frc/trajectory/Trajectory.h>
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
  void Periodic() override;

  void ResyncAbsRelEnc();
  void SetOverrideXboxInput(bool bOverride) { m_bOverrideXboxInput = bOverride; }
  void WheelsForward();
  void WheelsLeft();
  void WheelsBackward();
  void WheelsRight();

  //static constexpr units::meters_per_second_t kMaxSpeed = 3.0_mps;  // 3 meters per second
  //static constexpr units::radians_per_second_t kMaxAngularSpeed{std::numbers::pi};  // 1/2 rotation per second
// Safer sppeds for lab testing
  static constexpr units::meters_per_second_t kMaxSpeed = 1.0_mps;
  static constexpr units::radians_per_second_t kMaxAngularSpeed{0.25 * std::numbers::pi};

private:
  void SetAllDesiredState(const frc::SwerveModuleState& sms);

  static constexpr auto kTrackWidth = 20_in;
  static constexpr auto kWheelBase = 28_in;
  frc::Translation2d m_frontLeftLocation{kWheelBase / 2, kTrackWidth / 2};
  frc::Translation2d m_frontRightLocation{kWheelBase / 2, -kTrackWidth / 2};
  frc::Translation2d m_backLeftLocation{-kWheelBase / 2, kTrackWidth / 2};
  frc::Translation2d m_backRightLocation{-kWheelBase / 2, -kTrackWidth / 2};

//#define ZERO_OFFSETS
#ifdef ZERO_OFFSETS
  SwerveModule m_frontLeft { 1, 2, 0.000 };  SwerveModule m_frontRight { 3, 4, 0.000 };
  SwerveModule m_backRight { 5, 6, 0.000 };  SwerveModule m_backLeft   { 7, 8, 0.000 };
#else
  SwerveModule m_frontLeft { 1, 2, 0.440, false };  SwerveModule m_frontRight { 3, 4, 0.631, true };
  SwerveModule m_backLeft  { 7, 8, 0.960, false };  SwerveModule m_backRight  { 5, 6, 0.986, true };
#endif

  Gyro m_gyro;

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, 
      m_backLeftLocation, m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{
      m_kinematics,
      m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()}};

  bool m_bOverrideXboxInput = false;

  // Logging Member Variables
  frc::Timer m_timer;
  //std::vector<frc::Trajectory::State> m_StateHist;
  double m_velocity;
  double m_acceleration;

  wpi::log::DoubleLogEntry m_logRobotPoseX;
  wpi::log::DoubleLogEntry m_logRobotPoseY;
  wpi::log::DoubleLogEntry m_logRobotPoseTheta;
  wpi::log::DoubleLogEntry m_logRobotSpeed;
  wpi::log::DoubleLogEntry m_logRobotAccel;
};
