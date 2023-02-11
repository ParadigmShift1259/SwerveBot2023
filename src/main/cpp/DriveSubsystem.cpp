// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DriveSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative)
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  // m_StateHist.reserve(10000);
  // m_StateHist.clear();

  m_logRobotPoseX = wpi::log::DoubleLogEntry(log, "/odometry/robotPoseX");
  m_logRobotPoseY = wpi::log::DoubleLogEntry(log, "/odometry/robotPoseY");
  m_logRobotPoseTheta = wpi::log::DoubleLogEntry(log, "/odometry/robotPoseTheta");   
  m_logRobotSpeed = wpi::log::DoubleLogEntry(log, "/odometry/robotSpeed");
  m_logRobotAccel = wpi::log::DoubleLogEntry(log, "/odometry/robotAccel");
  m_logGyroPitch = wpi::log::DoubleLogEntry(log, "/gyro/pitch");


  frc::SmartDashboard::PutNumber("Input x speed", xSpeed.to<double>());
  frc::SmartDashboard::PutNumber("Input y speed", ySpeed.to<double>());
  frc::SmartDashboard::PutNumber("Input rot", rot.to<double>());

  if (m_bOverrideXboxInput == false)
  {
    auto states = m_kinematics.ToSwerveModuleStates(fieldRelative 
       ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
       : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

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
  UpdateOdometry();
  m_frontLeft.Periodic();
  m_frontRight.Periodic();
  m_backLeft.Periodic();
  m_backRight.Periodic();

  //Log Odometry Values
  frc::Pose2d pose = m_odometry.GetPose();
  frc::Trajectory::State state;
  state.t = m_timer.GetFPGATimestamp();
  state.pose = pose;
	//auto& prevState = m_StateHist.back();
  //state.velocity = (pose - prevState.pose).Translation().Norm() / (state.t - prevState.t);
  //state.acceleration = (state.velocity - prevState.velocity) / (state.t - prevState.t);
  //m_StateHist.push_back(state);

  m_velocity = (double)state.velocity;
  m_acceleration = (double)state.acceleration;

  m_logRobotPoseX.Append(pose.X().to<double>());
  m_logRobotPoseY.Append(pose.Y().to<double>());
  m_logRobotPoseTheta.Append(pose.Rotation().Degrees().to<double>());
  m_logRobotSpeed.Append(m_velocity);
  m_logRobotAccel.Append(m_acceleration);
  frc::SmartDashboard::PutNumber("GyroPitch", m_gyro.GetPitch());
  m_logGyroPitch.Append(m_gyro.GetPitch()); 
}

frc::Pose2d DriveSubsystem::GetPose()
{
  return m_odometry.GetPose();
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

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
  SwerveModulePositions modulePositions = {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                                           m_backLeft.GetPosition(), m_backRight.GetPosition()};

  m_odometry.ResetPosition(m_gyro.GetRotation2d(), modulePositions, pose);
}

void DriveSubsystem::SetHeading(units::degree_t heading)
{
  m_gyro.Set(heading);
}

void DriveSubsystem::WheelsForward()
{
  frc::SwerveModuleState sms;
  sms.angle = frc::Rotation2d{0.0_deg};
  sms.speed = -1.0_mps;
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

void DriveSubsystem::SetModuleStates(SwerveModuleStates desiredStates)
{
    m_kinematics.DesaturateWheelSpeeds(&desiredStates, kMaxSpeed);
    m_frontLeft.SetDesiredState(desiredStates[0]);
    m_frontRight.SetDesiredState(desiredStates[1]);
    m_backRight.SetDesiredState(desiredStates[3]);
    m_backLeft.SetDesiredState(desiredStates[2]);
}