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

static constexpr units::meters_per_second_t kMaxSpeed = 18.0_fps;  // L3 Gear Ratio Falcon Max Speed
static constexpr units::radians_per_second_t kMaxAngularSpeed{std::numbers::pi};  // 1/2 rotation per second
static constexpr units::radians_per_second_squared_t kMaxAngularAcceleration{4 * std::numbers::pi};  // 4 rotations per second squared

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
  void ResetOdometry(frc::Pose2d pose);
  void SetHeading(units::degree_t heading);
  void Periodic() override;
  double GetPitch() { return m_gyro.GetPitch(); }
  frc::Pose2d GetPose();
  /// Readable alias for array of swerve modules
  using SwerveModuleStates = wpi::array<frc::SwerveModuleState, 4>;
  using SwerveModulePositions = wpi::array<frc::SwerveModulePosition, 4>;
  void SetModuleStates(SwerveModuleStates desiredStates);

  void ResyncAbsRelEnc();
  void SetOverrideXboxInput(bool bOverride) { m_bOverrideXboxInput = bOverride; }
  void WheelsForward();
  void WheelsLeft();
  void WheelsBackward();
  void WheelsRight();

// Safer sppeds for lab testing
  // static constexpr units::meters_per_second_t kMaxSpeed = 1.0_mps;
  // static constexpr units::radians_per_second_t kMaxAngularSpeed{0.25 * std::numbers::pi};

private:
  void SetAllDesiredState(const frc::SwerveModuleState& sms);

  static constexpr auto kTrackWidth = 20_in;
  static constexpr auto kWheelBase = 28_in;
  const frc::Translation2d m_frontLeftLocation{kWheelBase / 2, kTrackWidth / 2};
  const frc::Translation2d m_frontRightLocation{kWheelBase / 2, -kTrackWidth / 2};
  const frc::Translation2d m_backLeftLocation{-kWheelBase / 2, kTrackWidth / 2};
  const frc::Translation2d m_backRightLocation{-kWheelBase / 2, -kTrackWidth / 2};

  // Mk4 swerve modules with L1 gear set
  // static constexpr double kFLoffset = 0.440;   static constexpr double kFRoffset = 0.631;
  // static constexpr double kBLoffset = 0.960;   static constexpr double kBRoffset = 0.986;

  // Mk4 swerve modules with L3 gear set
  static constexpr double kFLoffset = 0.002;    static constexpr double kFRoffset = 0.242;
  static constexpr double kBLoffset = 0.469;    static constexpr double kBRoffset = 0.762;

//#define ZERO_OFFSETS
#ifdef ZERO_OFFSETS
  SwerveModule m_frontLeft { 1, 2, 0.000, false };  SwerveModule m_frontRight { 3, 4, 0.000, true };
  SwerveModule m_backRight { 5, 6, 0.000, false };  SwerveModule m_backLeft   { 7, 8, 0.000, true };
#else
  SwerveModule m_frontLeft { 1, 2, kFLoffset, false };  SwerveModule m_frontRight { 3, 4, kFRoffset, true };
  SwerveModule m_backLeft  { 7, 8, kBLoffset, false };  SwerveModule m_backRight  { 5, 6, kBRoffset, true };
#endif

  Gyro m_gyro;

public:
  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, 
      m_backLeftLocation, m_backRightLocation};

private:
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
  wpi::log::DoubleLogEntry m_logGyroPitch;
};
