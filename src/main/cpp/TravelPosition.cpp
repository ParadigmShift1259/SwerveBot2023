#include "TravelPosition.h"

#include <frc2/command/WaitCommand.h>

#include "ConstantsDeploymentAngles.h"

TravelPosition::TravelPosition(ISubsystemAccess& subsystemAccess) 
  : m_claw(subsystemAccess.GetClaw())
  , m_deployment(subsystemAccess.GetDeployment())
  , m_timer()
{
  AddRequirements({&subsystemAccess.GetClaw(), &subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/travelPosition/startCommand");
}

void TravelPosition::Initialize()
{
  m_logStartCommand.Append(true);
  m_deployment.RetractBackPlate();
  m_deployment.RetractArm();
  m_claw.Close();
  m_timer.Reset();
  m_timer.Start();
}

void TravelPosition::Execute()
{
  if (m_timer.Get() > 0.5_s)
  {
    m_deployment.RotateArmToAngle(kTravelAngle);
  }
}

bool TravelPosition::IsFinished()
{
  return m_deployment.IsAtDegreeSetpoint(kTravelAngle);
}

void TravelPosition::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}