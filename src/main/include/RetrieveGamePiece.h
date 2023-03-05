#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "ISubsystemAccess.h"

using namespace frc2;

class RetrieveGamePiece : public frc2::CommandHelper<frc2::CommandBase, RetrieveGamePiece>
{
public:
    explicit RetrieveGamePiece(ISubsystemAccess& subsystemAccess);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    // ClawSubsystem& m_claw;
    // DeploymentSubsystem& m_deployment;
    // IntakeSubsystem& m_intake;

    //ISubsystemAccess& m_subsystemAccess;

    SequentialCommandGroup m_retrieveGamePiece;

    wpi::log::BooleanLogEntry m_logStartCommand;

    //bool m_isFinished = false;
};