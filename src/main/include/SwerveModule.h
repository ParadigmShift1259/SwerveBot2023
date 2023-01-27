// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include <string>

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DutyCycleEncoder.h>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>

#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;
using namespace rev;

class SwerveModule
{
public:
    SwerveModule(int driveMotorChannel, int turningMotorChannel, bool bInverted, double offset);
    frc::SwerveModuleState GetState();// const;
    frc::SwerveModulePosition GetPosition();// const;
    void SetDesiredState(const frc::SwerveModuleState& state);

private:
    static constexpr double kWheelRadius = 0.0508;
    static constexpr int kEncoderResolution = 4096;
    static constexpr double kTurnMotorRevsPerWheelRev = 12.8;

    static constexpr auto kModuleMaxAngularVelocity = std::numbers::pi * 1_rad_per_s;  // radians per second
    static constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2

    TalonFX m_driveMotor;
    CANSparkMax m_turningMotor;

    std::string m_id;

    SparkMaxAlternateEncoder m_turningEncoder = m_turningMotor.GetAlternateEncoder(SparkMaxAlternateEncoder::Type::kQuadrature, kEncoderResolution);//320);

    frc::DutyCycleEncoder m_absEnc;

    frc2::PIDController m_drivePIDController{1.0, 0, 0};
    
    SparkMaxPIDController m_turningPIDController = m_turningMotor.GetPIDController();

    frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V, 3_V / 1_mps};
    // frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{1_V, 0.5_V / 1_rad_per_s};
};
