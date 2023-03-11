#pragma once

#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class TravelPosition : public frc2::CommandHelper<frc2::CommandBase, TravelPosition>
{
public:
    explicit TravelPosition(ISubsystemAccess& subsystemAccess);

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