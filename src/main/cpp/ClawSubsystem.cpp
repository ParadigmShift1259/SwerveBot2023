#include "ClawSubsystem.h"

using namespace frc;

ClawSubsystem::ClawSubsystem()
    : m_solenoid(PneumaticsModuleType::CTREPCM, kClawSolenoid)
{
    m_solenoid.Set(false);
}

void ClawSubsystem::Periodic()
{
}

void ClawSubsystem::Open()
{
    m_solenoid.Set(true);
}

void ClawSubsystem::Close()
{
    m_solenoid.Set(false);
}
