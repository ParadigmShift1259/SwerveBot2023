#include "RotateTurntableCCW.h"

#include <frc2/command/WaitCommand.h>

RotateTurntableCCW::RotateTurntableCCW(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
  , m_turntable(subsystemAccess.GetTurntable())
{
  AddRequirements({&subsystemAccess.GetDeployment(), &subsystemAccess.GetTurntable()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/RotateTurntableCCW/startCommand");
  m_logStartCommand.Append(true);
}


void RotateTurntableCCW::Initialize()
{
  m_deployment.ExtendBackPlate();
  frc2::WaitCommand(0.25_s); // Wait for backplate to extend and turntable motor to engage
  m_turntable.SetTurnTable(kTurntableCCWSpeed);
}

void RotateTurntableCCW::Execute() 
{
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