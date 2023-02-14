#include "ClawSubsystem.h"

using namespace std;
using namespace frc;

ClawSubsystem::ClawSubsystem()
    : m_solenoid(PneumaticsModuleType::REVPH, 0)//kSolenoidPort)
{

}

void ClawSubsystem::Periodic()
{
    
}