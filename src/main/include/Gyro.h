#pragma once

#include <ctre/phoenix.h>

class Gyro
{
public:
    Gyro();

    frc::Rotation2d GetRotation2d();

    void Reset();

    PigeonIMU m_gyro;
};