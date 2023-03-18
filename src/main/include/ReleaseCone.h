#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"
#include "ConstantsDeploymentPositions.h"
#include "ConstantsDeploymentAbsolutes.h"

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

    double m_releasePosition = kTravelPosition; // Initialize to something safe
    double m_releaseAbsolute = kTravelAbsolute; // Initialize to something safe

    wpi::log::BooleanLogEntry m_logStartCommand;
    wpi::log::DoubleLogEntry m_logPosition;
};