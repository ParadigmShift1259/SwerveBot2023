#include "TurntableSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace std;
using namespace frc;

TurntableSubsystem::TurntableSubsystem()
    : m_turntablemotor(0)//kTurnTablePort)
{
    m_turntablemotor.SetNeutralMode(NeutralMode::Brake);
    m_turntablemotor.SetInverted(true);//kTurnTableInverted);
    m_turntablemotor.ConfigOpenloopRamp(0.0, 0);//kTurnTableRampRate, kTimeout);
}

void TurntableSubsystem::Periodic()
{

}

void TurntableSubsystem::SetTurnTable(double speed)
{
    m_turntablemotor.Set(ControlMode::PercentOutput, speed);
}