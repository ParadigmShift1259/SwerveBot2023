// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

#include <frc/smartdashboard/SmartDashboard.h> //temp

SwerveModule::SwerveModule(const int driveMotorChannel, const int turningMotorChannel, bool bInverted, double offset)
  : m_driveMotor(driveMotorChannel)
  , m_turningMotor(turningMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless)
  , m_id(std::to_string(turningMotorChannel / 2))
  , m_absEnc((turningMotorChannel / 2) - 1)
{
  m_absEnc.SetDutyCycleRange(1.0/4096.0, 4095.0/4096.0); 
  m_absEnc.SetPositionOffset(offset);
  auto angle = m_absEnc.Get();
  frc::SmartDashboard::PutData("Abs Enc" + m_id, &m_absEnc);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.
  m_turningEncoder.SetInverted(bInverted);
  m_turningEncoder.SetPositionConversionFactor(2.0 * std::numbers::pi / kTurnMotorRevsPerWheelRev);
  //m_turningEncoder.SetPosition(angle.to<double>());
  //m_turningEncoder.SetPosition(0.5);

  m_driveMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
  m_driveMotor.SetSelectedSensorPosition(0.0);

  m_turningPIDController.SetFeedbackDevice(m_turningEncoder);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.SetOutputRange(-1.0, 1.0);
  constexpr double kTurnP = 0.075;
  constexpr double kTurnI = 0.000001;
  constexpr double kTurnD = 0.0;
  m_turningPIDController.SetP(kTurnP);
  m_turningPIDController.SetI(kTurnI);
  m_turningPIDController.SetD(kTurnD);
  m_turningMotor.SetSmartCurrentLimit(20);
}

void SwerveModule::Periodic()
{
  auto angle = m_absEnc.Get();
  frc::SmartDashboard::PutNumber("Abs Angle" + m_id, angle.to<double>());
  frc::SmartDashboard::PutNumber("Turn Enc Pos" + m_id, m_turningEncoder.GetPosition());
  //printf("Get returns %.3f GetDist returns %.3f\n", angle.to<double>(), m_absEnc.GetDistance());
}

frc::SwerveModuleState SwerveModule::GetState()// const
{
  return {units::meters_per_second_t{CalcMetersPerSec()},
          units::radian_t{m_turningEncoder.GetPosition()}};
}

units::meters_per_second_t SwerveModule::CalcMetersPerSec()
{
   double ticksPer100ms = m_driveMotor.GetSelectedSensorVelocity();
   return units::meters_per_second_t(kDriveEncoderMetersPerSec * ticksPer100ms);
}

double SwerveModule::CalcTicksPer100Ms(units::meters_per_second_t speed)
{
   return speed.to<double>() / kDriveEncoderMetersPerSec;
}

frc::SwerveModulePosition SwerveModule::GetPosition() //const
{
  return {units::meter_t{m_driveMotor.GetSelectedSensorPosition()}, // TODO raw sensor units
          units::radian_t{m_turningEncoder.GetPosition()}};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(referenceState, frc::Rotation2d{ units::radian_t(m_turningEncoder.GetPosition()) });
  frc::SmartDashboard::PutNumber("Turn Encoder Position" + m_id, m_turningEncoder.GetPosition()); //temp

  // Calculate the drive output from the drive PID controller.
  // const auto driveOutput = m_drivePIDController.Calculate(m_driveMotor.GetSelectedSensorVelocity(), state.speed.value());

  // const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed); Not Used

  // Calculate the turning motor output from the turning PID controller.
  m_turningPIDController.SetReference(state.angle.Radians().to<double>(), CANSparkMax::ControlType::kPosition);
  // m_turningPIDController.SetReference(0.1, CANSparkMax::ControlType::kPosition);

  // const auto turnFeedforward = m_turnFeedforward.Calculate(m_turningPIDController.GetSetpoint().velocity);

  // Set the motor outputs.
  // m_driveMotor.Set(ControlMode::Velocity, driveOutput);
}