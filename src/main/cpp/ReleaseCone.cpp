
#include "ReleaseCone.h"

//#include <frc/smartdashboard/SmartDashboard.h>

#include "ConstantsDeploymentAngles.h"

ReleaseCone::ReleaseCone(ISubsystemAccess& subsystemAccess)
  : m_claw(subsystemAccess.GetClaw())
  , m_deployment(subsystemAccess.GetDeployment())
  , m_releaseAngle(kTravelAngle) // Initialize it to something safe
{
  AddRequirements({&subsystemAccess.GetClaw(), &subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/ReleaseCone/startCommand");
  m_logAngle = wpi::log::DoubleLogEntry(log, "/ReleaseCone/angle");
}

void ReleaseCone::Initialize()
{
  m_logStartCommand.Append(true);

  if (m_deployment.IsAtDegreeSetpoint(kPlaceHighAngle))
  {
    m_releaseAngle = kReleaseHighAngle;
  }
  else if (m_deployment.IsAtDegreeSetpoint(kPlaceLowAngle))
  {
    m_releaseAngle = kReleaseLowAngle;
  }
  else
  {
    printf("NOT AT PLACE ANGlE");
    return;
  }
  m_deployment.RotateArmToAngle(m_releaseAngle);
  m_logAngle.Append(m_releaseAngle.to<double>());
}

void ReleaseCone::Execute()
{

}

bool ReleaseCone::IsFinished()
{
  return m_deployment.IsAtDegreeSetpoint(m_releaseAngle);
}

void ReleaseCone::End(bool interrupted)
{
  if (IsFinished() && !interrupted)
  {
    m_claw.Open();
  }
  m_logStartCommand.Append(false);
}