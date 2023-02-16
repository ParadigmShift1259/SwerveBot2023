#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class IntakeDeploy : public frc2::CommandHelper<frc2::CommandBase, IntakeDeploy> {
 public:
  explicit IntakeDeploy(ISubsystemAccess& subsystemAccess);

  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished();
  
 private:
  IntakeSubsystem& m_intake;
  bool m_bRunning = false;
};