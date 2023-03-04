#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class PlaceHighCube : public frc2::CommandHelper<frc2::CommandBase, PlaceHighCube>
{
public:
    explicit PlaceHighCube(ISubsystemAccess& subsystemAccess);

    void Execute() override;
    bool IsFinished() override;

    void End(bool interrupted) override;

private:
    DeploymentSubsystem& m_deployment;

    wpi::log::BooleanLogEntry m_logStartCommand;
    wpi::log::DoubleLogEntry m_logAngle;
};