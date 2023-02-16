#pragma once

#include "ClawSubsystem.h"
#include "DeploymentSubsystem.h"
#include "DriveSubsystem.h"
#include "IntakeSubsystem.h"
#include "TurntableSubsystem.h"
#include "VisionSubsystem.h"

class ISubsystemAccess
{
public:
    virtual ClawSubsystem&          GetClaw() = 0;
    virtual DeploymentSubsystem&    GetDeployment() = 0;
    virtual DriveSubsystem&         GetDrive() = 0;
    virtual IntakeSubsystem&        GetIntake() = 0;
    virtual TurntableSubsystem&     GetTurntable() = 0;
    virtual VisionSubsystem&        GetVision() = 0;
};
