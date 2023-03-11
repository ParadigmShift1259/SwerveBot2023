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
  frc2::WaitCommand(1.0_s); // Wait for backplate to extend and turntable motor to engage
}

void RotateTurntableCW::Execute() 
{
  m_turntable.SetTurnTable(kTurntableCWSpeed);
}

void RotateTurntableCW::End(bool interrupted) 
{
    m_turntable.SetTurnTable(0.0);
    m_logStartCommand.Append(false);
}