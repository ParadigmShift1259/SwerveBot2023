#include "RotateTurntableCCW.h"

#include <frc2/command/WaitCommand.h>

RotateTurntableCCW::RotateTurntableCCW(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
  , m_turntable(subsystemAccess.GetTurntable())
{
  AddRequirements({&subsystemAccess.GetDeployment(), &subsystemAccess.GetTurntable()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/rotateTurntableCCW/startCommand");
}

void RotateTurntableCCW::Initialize()
{
  m_logStartCommand.Append(true);
  m_deployment.ExtendBackPlate();
  m_timer.Reset();
  m_timer.Start();
}

void RotateTurntableCCW::Execute() 
{
  if (m_timer.Get() > 0.5_s)
  {
    m_turntable.SetTurnTable(kTurntableCWSpeed);
  }
}

void RotateTurntableCCW::End(bool interrupted) 
{
  m_turntable.SetTurnTable(0.0);
  m_logStartCommand.Append(false);
}