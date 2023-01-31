// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(const int driveMotorCanId, const int turningMotorCanId, double offset)
  : m_driveMotor(driveMotorCanId)
  , m_turningMotor(turningMotorCanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless)
  , m_id(std::to_string(turningMotorCanId / 2))
  , m_absEnc((turningMotorCanId / 2) - 1)
  , m_offset(offset)
{
  m_absEnc.SetConnectedFrequencyThreshold(200);
  //m_absEnc.SetDutyCycleRange(1.0/4096.0, 4095.0/4096.0); 
  //m_absEnc.Reset(); 
  //m_absEnc.SetPositionOffset(offset);
  auto angle = m_absEnc.GetAbsolutePosition();

  m_turningEncoder.SetInverted(true);               // SDS Mk4i motors are mounted upside down compared to the Mk4
  //m_turningEncoder.SetPositionConversionFactor(1.0);
  m_turningEncoder.SetPositionConversionFactor(2.0 * std::numbers::pi);
  //m_turningEncoder.SetPosition(angle.to<double>() * 2.0 * std::numbers::pi);
  m_turningEncoder.SetPosition((angle - m_offset) * 2 * std::numbers::pi);

  m_driveMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
  m_driveMotor.SetSelectedSensorPosition(0.0);

  m_turningPIDController.SetFeedbackDevice(m_turningEncoder);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.SetOutputRange(-1.0, 1.0);
  constexpr double kTurnP = 1.0;
  constexpr double kTurnI = 0.000001;
  constexpr double kTurnD = 0.0;
  m_turningPIDController.SetP(kTurnP);
  m_turningPIDController.SetI(kTurnI);
  m_turningPIDController.SetD(kTurnD);
  frc::SmartDashboard::PutBoolean("Load Turn PID", false);
  frc::SmartDashboard::PutNumber("Turn P", kTurnP);
  frc::SmartDashboard::PutNumber("Turn I", kTurnI);
  frc::SmartDashboard::PutNumber("Turn D", kTurnD);
  m_turningMotor.SetSmartCurrentLimit(20);
  m_turningMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);

  m_drivePIDController.SetP(0.5);
  
  m_timer.Reset();
  m_timer.Start();
}

void SwerveModule::Periodic()
{
  auto time = m_timer.Get();
  if (time < 1.0_s)
  {
    ResyncAbsRelEnc();
  }

  bool bLoadPID = frc::SmartDashboard::GetBoolean("Load Turn PID", false);
  if (bLoadPID)
  {
    double kTurnP = frc::SmartDashboard::GetNumber("Turn P", 0.5);
    double kTurnI = frc::SmartDashboard::GetNumber("Turn I", 0.00001);
    double kTurnD = frc::SmartDashboard::GetNumber("Turn D", 0.05);
    m_turningPIDController.SetP(kTurnP);
    m_turningPIDController.SetI(kTurnI);
    m_turningPIDController.SetD(kTurnD);
    frc::SmartDashboard::PutNumber("Turn P echo", kTurnP);
    frc::SmartDashboard::PutNumber("Turn I echo", kTurnI);
    frc::SmartDashboard::PutNumber("Turn D echo", kTurnD);
    printf("Loaded Turn PID values P %.3f I %.3f D %.3f\n", kTurnP, kTurnI, kTurnD);
    //frc::SmartDashboard::PutBoolean("Load Turn PID", false);
  }

  auto absPos = m_absEnc.GetAbsolutePosition();
  frc::SmartDashboard::PutNumber("Abs Pos" + m_id, absPos);
  frc::SmartDashboard::PutNumber("Abs Pos Offset" + m_id, absPos - m_offset);

  //frc::SmartDashboard::PutNumber("Turn Enc Pos" + m_id, m_turningEncoder.GetPosition() * 2 * std::numbers::pi);
  //frc::SmartDashboard::PutNumber("Turn Mot Pos" + m_id, m_turningEncoder.GetPosition() * 2 * std::numbers::pi * kTurnMotorRevsPerWheelRev / (2 * std::numbers::pi));
  frc::SmartDashboard::PutNumber("Turn Enc Pos" + m_id, m_turningEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Turn Mot Pos" + m_id, m_turningEncoder.GetPosition() * kTurnMotorRevsPerWheelRev / (2 * std::numbers::pi));
}

void SwerveModule::ResyncAbsRelEnc()
{
  auto time = m_timer.Get();
  auto angle = m_absEnc.GetAbsolutePosition();
  m_turningEncoder.SetPosition((angle - m_offset) * 2 * std::numbers::pi);
  printf("%.3f abs enc set pos\n", angle - m_offset);
  printf("Module %s %.3f Set abs enc %.3f [rot] %.3f [rad] to rel enc %.3f [rad] mot pos %.3f [rad]\n"
        , m_id.c_str()
        , time.to<double>()
        , angle
        , angle * 2 * std::numbers::pi
        //, m_turningEncoder.GetPosition() * 2 * std::numbers::pi
        //, m_turningEncoder.GetPosition() * 2 * std::numbers::pi * kTurnMotorRevsPerWheelRev / (2 * std::numbers::pi));
        , m_turningEncoder.GetPosition()
        , m_turningEncoder.GetPosition() * kTurnMotorRevsPerWheelRev / (2 * std::numbers::pi));
}

frc::SwerveModuleState SwerveModule::GetState()
{
  return {units::meters_per_second_t{ CalcMetersPerSec() },
          units::radian_t{ m_turningEncoder.GetPosition() } };
          //units::radian_t{ m_turningEncoder.GetPosition() * 2 * std::numbers::pi } };
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

frc::SwerveModulePosition SwerveModule::GetPosition()
{
  return {units::meter_t{ m_driveMotor.GetSelectedSensorPosition() }, // TODO raw sensor units
          units::radian_t{ m_turningEncoder.GetPosition() } };
          //units::radian_t{ m_turningEncoder.GetPosition() * 2 * std::numbers::pi } };
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState)
{
  // Optimize the reference state to avoid spinning further than 90 degrees
  //double currPosition = m_turningEncoder.GetPosition() * 2 * std::numbers::pi;
  double currPosition = m_turningEncoder.GetPosition();
  const auto state = frc::SwerveModuleState::Optimize(referenceState, frc::Rotation2d{ units::radian_t(currPosition) });
  frc::SmartDashboard::PutNumber("Turn Enc Pos" + m_id, currPosition);
  frc::SmartDashboard::PutNumber("Turn Mot Pos" + m_id, currPosition * kTurnMotorRevsPerWheelRev / (2 * std::numbers::pi));

  // Calculate the drive output from the drive PID controller.
  //const auto driveOutput = m_drivePIDController.Calculate(m_driveMotor.GetSelectedSensorVelocity(), state.speed.value());
  const auto driveOutput = m_drivePIDController.Calculate(CalcMetersPerSec().to<double>(), state.speed.value());

  // const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed); Not Used

  // Calculate the turning motor output from the turning PID controller.
  frc::SmartDashboard::PutNumber("Turn Ref Opt" + m_id, state.angle.Radians().to<double>());
  frc::SmartDashboard::PutNumber("Turn Ref" + m_id, referenceState.angle.Radians().to<double>());
  double newRef = state.angle.Radians().to<double>();
  //double newRef = state.angle.Radians().to<double>() * kTurnMotorRevsPerWheelRev / (2 * std::numbers::pi);
  // double newRef = referenceState.angle.Radians().to<double>() * kTurnMotorRevsPerWheelRev / (2 * std::numbers::pi);
  frc::SmartDashboard::PutNumber("Turn Ref Motor" + m_id, newRef);
  m_turningPIDController.SetReference(newRef, CANSparkMax::ControlType::kPosition);

  // const auto turnFeedforward = m_turnFeedforward.Calculate(m_turningPIDController.GetSetpoint().velocity);

  // Set the motor outputs.
  m_driveMotor.Set(ControlMode::Velocity, driveOutput);
}
