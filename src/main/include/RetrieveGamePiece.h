#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class RetrieveGamePiece : public frc2::CommandHelper<frc2::CommandBase, RetrieveGamePiece>
{
public:
    explicit RetrieveGamePiece(ISubsystemAccess& subsystemAccess);

    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    ClawSubsystem& m_claw;
    DeploymentSubsystem& m_deployment;
    IntakeSubsystem& m_intake;

    ISubsystemAccess& m_subsystemAccess;

    wpi::log::BooleanLogEntry m_logStartCommand;
};