#include "Gyro.h"

Gyro::Gyro() :
    m_gyro(0)
{

}

frc::Rotation2d Gyro::GetRotation2d()
{
    auto retVal = std::remainder(m_gyro.GetFusedHeading(), 360.0);
    if (retVal > 180.0)
        retVal -= 360.0;

    return frc::Rotation2d(units::degree_t(retVal));
}

void Gyro::Reset()
{
    m_gyro.SetFusedHeading(0.0);
}