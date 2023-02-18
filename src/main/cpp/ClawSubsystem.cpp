#include "ClawSubsystem.h"

using namespace std;
using namespace frc;

ClawSubsystem::ClawSubsystem()
    : m_solenoid(PneumaticsModuleType::REVPH, kClawSolenoid)
{

}

void ClawSubsystem::Periodic()
{
    
}

// TODO Check claw solenoid if open is false and close is true
void ClawSubsystem::Open()
{
    m_solenoid.Set(false);
}

void ClawSubsystem::Close()
{
    m_solenoid.Set(true);
}
