#include "RotateTurntableCCW.h"

RotateTurntableCCW::RotateTurntableCCW(ISubsystemAccess& subsystemAccess) 
  : m_turntable(subsystemAccess.GetTurntable())
{
  AddRequirements({&subsystemAccess.GetTurntable()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/RotateTurntableCCW/startCommand");
  m_logStartCommand.Append(true);
}

void RotateTurntableCCW::Initialize()
{
  m_timer.Reset();
  m_timer.Start();
}

void RotateTurntableCCW::Execute() 
{
  m_turntable.SetTurnTable(kTurntableCCWSpeed);
}

bool RotateTurntableCCW::IsFinished()
{
  return true;
}

void RotateTurntableCCW::End(bool interrupted) 
{
  m_turntable.SetTurnTable(0.0);
  m_logStartCommand.Append(false);
}