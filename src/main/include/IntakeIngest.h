#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class IntakeIngest : public frc2::CommandHelper<frc2::CommandBase, IntakeIngest>
{
public:
  explicit IntakeIngest(ISubsystemAccess& subsystemAccess);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  
private:
  DeploymentSubsystem& m_deployment;
  IntakeSubsystem& m_intake;
  TurntableSubsystem& m_turntable;

  wpi::log::BooleanLogEntry m_logStartCommand;
};