#include "RotateTurntableCW.h"

RotateTurntableCW::RotateTurntableCW(ISubsystemAccess& subsystemAccess) 
  : m_turntable(subsystemAccess.GetTurntable())
{
  AddRequirements({&subsystemAccess.GetTurntable()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/rotateTurntableCW/startCommand");
}

void RotateTurntableCW::Initialize()
{
    m_timer.Reset();
    m_timer.Start();
}

void RotateTurntableCW::Execute() 
{
    m_turntable.SetTurnTable(kTurntableCWSpeed);
}

bool RotateTurntableCW::IsFinished()
{
    return m_timer.Get() > kTurntableCWRunTime;
}

void RotateTurntableCW::End(bool interrupted) 
{
    m_turntable.SetTurnTable(0.0);
    m_logStartCommand.Append(false);
}