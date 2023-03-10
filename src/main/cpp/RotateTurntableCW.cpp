#include "RotateTurntableCW.h"

#include <frc2/command/WaitCommand.h>

RotateTurntableCW::RotateTurntableCW(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
  , m_turntable(subsystemAccess.GetTurntable())
{
  AddRequirements({&subsystemAccess.GetDeployment(), &subsystemAccess.GetTurntable()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/rotateTurntableCW/startCommand");
}

void RotateTurntableCW::Initialize()
{
  m_logStartCommand.Append(true);
  m_deployment.ExtendBackPlate();
  frc2::WaitCommand(0.25_s); // Wait for backplate to extend and turntable motor to engage
  m_turntable.SetTurnTable(kTurntableCWSpeed);
}

void RotateTurntableCW::Execute() 
{
}

bool RotateTurntableCW::IsFinished()
{
  return true;
}

void RotateTurntableCW::End(bool interrupted) 
{
    m_turntable.SetTurnTable(0.0);
    m_logStartCommand.Append(false);
}