#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

#include "ConstantsCANIDs.h"
#include "ConstantsDigitalOut.h"

using namespace rev;
using namespace frc;

constexpr double kClawIngestSpeed = 0.9;
constexpr double kClawReleaseSpeed = -0.2;
constexpr double kClawHoldSpeed = 0.2;

class ClawSubsystem : public frc2::SubsystemBase
{
public:
    ClawSubsystem();
    void Periodic();
    void Ingest();
    void Release();
    void Hold();
    void Stop();
    bool IsPhotoeyeActive();
        
private:
    CANSparkMax m_motor;
    DigitalInput m_photoeye;
};