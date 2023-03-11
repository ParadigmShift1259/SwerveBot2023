#pragma once

#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class PopHookPosition : public frc2::CommandHelper<frc2::CommandBase, PopHookPosition>
{
public:
    explicit PopHookPosition(ISubsystemAccess& subsystemAccess);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;
     
private:
    ClawSubsystem& m_claw;
    DeploymentSubsystem& m_deployment;
    frc::Timer m_timer;

    wpi::log::BooleanLogEntry m_logStartCommand;
};