#include "RetrievePosition.h"

#include "ConstantsDeploymentAngles.h"

#include <frc/smartdashboard/SmartDashboard.h>

RetrievePosition::RetrievePosition(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/retrievePosition/startCommand");
}

void RetrievePosition::Execute()
{
  // degree_t angle = degree_t(SmartDashboard::GetNumber("GotoAngle", 7.0));
  m_deployment.RotateArmToAngle(kRetrieveAngle);
  // m_deployment.RotateArmToAngle(angle);
}

bool RetrievePosition::IsFinished()
{
  return m_deployment.IsAtDegreeSetpoint(kRetrieveAngle);
  // return m_deployment.IsAtDegreeSetpoint(degree_t(SmartDashboard::GetNumber("GotoAngle", 7.0)));
}

void RetrievePosition::End(bool interrupted)
{
  //m_deployment.ExtendArm();
  m_logStartCommand.Append(false);
}