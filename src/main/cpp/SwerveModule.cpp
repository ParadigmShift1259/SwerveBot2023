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

  m_driveMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.
  m_turningEncoder.SetInverted(bInverted);
  m_turningEncoder.SetPositionConversionFactor(2 * std::numbers::pi / kTurnMotorRevsPerWheelRev);
  m_turningEncoder.SetPosition(0.0);

  m_driveMotor.SetSelectedSensorPosition(0.0);
  m_turningPIDController.SetFeedbackDevice(m_turningEncoder);

  m_turningMotor.SetSmartCurrentLimit(20);
  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.SetOutputRange(-1.0, 1.0);

  m_absEnc.SetDutyCycleRange(1.0/4096.0, 4095.0/4096.0); 
  m_absEnc.SetPositionOffset(offset);///360.0);
  //m_absEnc.SetDistancePerRotation(360.0);
  auto angle = m_absEnc.Get();
  frc::SmartDashboard::PutNumber("Abs Angle" + m_id, angle.to<double>());
  frc::SmartDashboard::PutData("Abs Enc" + m_id, &m_absEnc);
}

frc::SwerveModuleState SwerveModule::GetState()// const
{
  return {units::meters_per_second_t{m_driveMotor.GetSelectedSensorVelocity()},
          units::radian_t{m_turningEncoder.GetPosition()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() //const
{
  return {units::meter_t{m_driveMotor.GetSelectedSensorPosition()},
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