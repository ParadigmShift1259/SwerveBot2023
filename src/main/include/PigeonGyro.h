#pragma once

#include <units/angular_velocity.h>
#include <ctre/phoenix.h>

#define USE_PIGEON_2
#ifdef USE_PIGEON_2
#include <ctre/phoenix/sensors/Pigeon2.h>
#endif

class PigeonGyro
{
public:
    PigeonGyro();

    frc::Rotation2d GetRotation2d();
    double GetPitch() { return m_gyro.GetPitch(); }
    void Reset();
    void Set(units::degree_t yaw);
    units::degrees_per_second_t GetTurnRate();
         
#ifdef USE_PIGEON_2
    Pigeon2 m_gyro;
#else
    PigeonIMU m_gyro;
#endif
};