#include "TravelPosition.h"

#include <frc2/command/WaitCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

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
  m_deployment.RetractArm();

  // if (!m_claw.IsPhotoeyeActive())
  // {
  //   m_claw.Stop();
  // }

  // m_deployment.RotateArmSetAbsPos(kTravelAbsolute);

  m_timer.Reset();
  m_timer.Start();
}

void TravelPosition::Execute()
{
  SmartDashboard::PutNumber("TravTimer", m_timer.Get().to<double>());
  if (m_timer.Get() > 0.5_s)
  {
    m_deployment.RotateArm(kTravelPosition);
  }
}

bool TravelPosition::IsFinished()
{
  return m_deployment.IsAtAbsoluteSetpoint(kTravelAbsolute);
}

void TravelPosition::End(bool interrupted)
{
  m_deployment.ResetEncoder(kTravelPosition);
  m_logStartCommand.Append(false);
}