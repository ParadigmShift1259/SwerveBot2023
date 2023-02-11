#include "Gyro.h"

Gyro::Gyro() :
    m_gyro(1)
{

}

frc::Rotation2d Gyro::GetRotation2d()
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

void Gyro::Reset()
{
#ifdef USE_PIGEON_2
    m_gyro.SetYaw(0.0);
#else
    m_gyro.SetFusedHeading(0.0);
#endif
}

void Gyro::Set(units::degree_t yaw)
{
    m_gyro.SetYaw(yaw.to<double>());
}