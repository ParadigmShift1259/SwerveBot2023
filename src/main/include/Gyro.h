#pragma once

#include <ctre/phoenix.h>

#define USE_PIGEON_2
#ifdef USE_PIGEON_2
#include <ctre/phoenix/sensors/Pigeon2.h>
#endif

class Gyro
{
public:
    Gyro();

    frc::Rotation2d GetRotation2d();
    double GetPitch() { return m_gyro.GetPitch(); }
    void Reset();
         
#ifdef USE_PIGEON_2
    Pigeon2 m_gyro;
#else
    PigeonIMU m_gyro;
#endif
};