#include "TravelPosition.h"

#include <frc2/command/WaitCommand.h>

#include "ConstantsDeploymentPositions.h"
#include "ConstantsDeploymentAbsolutes.h"

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

  if (!m_claw.IsPhotoeyeActive())
  {
    m_claw.Stop();
  }

  m_timer.Reset();
  m_timer.Start();
}

void TravelPosition::Execute()
{
  if (m_timer.Get() > 0.5_s)
  {
    m_deployment.RotateArm(kTravelPosition);
    m_deployment.ResetEncoderWithAbsolute();
  }
}

bool TravelPosition::IsFinished()
{
  return m_deployment.IsAtSetpoint(kTravelAbsolute);
}

void TravelPosition::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}