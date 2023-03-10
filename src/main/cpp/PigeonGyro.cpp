#include "PigeonGyro.h"

PigeonGyro::PigeonGyro() :
    m_gyro(1)
{

}

frc::Rotation2d PigeonGyro::GetRotation2d()
{
#ifdef USE_PIGEON_2
    auto retVal = std::remainder(m_gyro.GetYaw(), 360.0);
#else
    auto retVal = std::remainder(m_gyro.GetFusedHeading(), 360.0);
#endif
    if (retVal > 180.0)
        retVal -= 360.0;
    return frc::Rotation2d(units::degree_t(retVal));
}

void PigeonGyro::Reset()
{
#ifdef USE_PIGEON_2
    m_gyro.SetYaw(0.0);
#else
    m_gyro.SetFusedHeading(0.0);
#endif
}

void PigeonGyro::Set(units::degree_t yaw)
{
    m_gyro.SetYaw(yaw.to<double>());
}

units::degrees_per_second_t PigeonGyro::GetTurnRate()
{
    double turnRates [3] = {0, 0, 0};
    m_gyro.GetRawGyro(turnRates);
    return units::degrees_per_second_t(turnRates[2]); 
}