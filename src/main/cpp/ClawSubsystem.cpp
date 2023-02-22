#include "ClawSubsystem.h"

using namespace frc;

ClawSubsystem::ClawSubsystem()
    : m_solenoid(PneumaticsModuleType::REVPH, kClawSolenoid)
{

}

void ClawSubsystem::Periodic()
{
    
}

void ClawSubsystem::Open()
{
    m_solenoid.Set(false);
}

void ClawSubsystem::Close()
{
    m_solenoid.Set(true);
}
