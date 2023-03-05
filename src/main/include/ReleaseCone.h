#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class ReleaseCone : public frc2::CommandHelper<frc2::CommandBase, ReleaseCone>
{
public:
    explicit ReleaseCone(ISubsystemAccess& subsystemAccess);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

    void End(bool interrupted) override;

private:
    ClawSubsystem& m_claw;
    DeploymentSubsystem& m_deployment;

    degree_t m_releaseAngle;

    wpi::log::BooleanLogEntry m_logStartCommand;
    wpi::log::DoubleLogEntry m_logAngle;
};