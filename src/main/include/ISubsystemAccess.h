#pragma once

#ifndef __SUBSYSTEMACCESS_H__
#define __SUBSYSTEMACCESS_H__

#include "ClawSubsystem.h"
#include "DeploymentSubsystem.h"
#include "IDriveSubsystem.h"
#include "IntakeSubsystem.h"
#include "TurntableSubsystem.h"
#include "VisionSubsystem.h"

class ISubsystemAccess
{
public:
    virtual ClawSubsystem&          GetClaw() = 0;
    virtual DeploymentSubsystem&    GetDeployment() = 0;
    virtual IDriveSubsystem&        GetDrive() = 0;
    virtual IntakeSubsystem&        GetIntake() = 0;
    virtual TurntableSubsystem&     GetTurntable() = 0;
    virtual VisionSubsystem&        GetVision() = 0;
};

#endif  //ndef __SUBSYSTEMACCESS_H__
