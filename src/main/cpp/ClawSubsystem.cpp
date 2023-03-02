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
  printf("ClawSubsystem::Open() m_solenoid.Get() %d\n", m_solenoid.Get());
    m_solenoid.Set(false);
}

void ClawSubsystem::Close()
{
    m_solenoid.Set(true);
}
