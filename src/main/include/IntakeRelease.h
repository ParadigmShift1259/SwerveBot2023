#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class IntakeRelease : public frc2::CommandHelper<frc2::CommandBase, IntakeRelease> 
{
public:
  explicit IntakeRelease(ISubsystemAccess& subsystemAccess);

  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:
  DeploymentSubsystem& m_deployment;
  IntakeSubsystem& m_intake;

  wpi::log::BooleanLogEntry m_logStartCommand;
};