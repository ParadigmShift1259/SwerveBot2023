#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class IntakeDeploy : public frc2::CommandHelper<frc2::CommandBase, IntakeDeploy> {
public:
  explicit IntakeDeploy(ISubsystemAccess& subsystemAccess);

  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  IntakeSubsystem& m_intake;

  wpi::log::BooleanLogEntry m_logStartCommand;
};