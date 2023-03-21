#pragma once

#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

#include "ConstantsCANIDs.h"
#include "ConstantsDigitalOut.h"

using namespace rev;
using namespace frc;

constexpr double kClawIngestSpeed = 0.9;
constexpr double kClawIngestCubeSpeed = 0.4;
constexpr double kClawReleaseSpeed = -0.2;
constexpr double kClawHoldSpeed = 0.2;

class ClawSubsystem : public frc2::SubsystemBase
{
public:
    ClawSubsystem();
    void Periodic();
    void Ingest();
    void IngestCube();
    void Release();
    void Hold();
    void Stop();
    bool IsPhotoeyeActive();
    void Unclamp();
    void Clamp();
        
private:
    CANSparkMax m_motor;
    SparkMaxRelativeEncoder m_enc = m_motor.GetEncoder();
    DigitalInput m_photoeye;
    frc::Solenoid m_solenoid;
};