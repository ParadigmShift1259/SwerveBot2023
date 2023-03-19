
#pragma once

#include <wpi/DataLog.h>

#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <frc/DataLogManager.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

#include "ConstantsDigitalOut.h"
#include "ConstantsCANIDs.h"

#include <units/angle.h>
#include <units/angular_velocity.h>

using namespace rev;
using namespace units;

class DeploymentSubsystem : public frc2::SubsystemBase 
{

    public:
        DeploymentSubsystem();

        /// Will be called periodically whenever the CommandScheduler runs.
        void Periodic() override;

        /// Drives the deployment arm to a specific angle
        /// \param rotations  Desired rotation of the motor shaft to rotate to; see ConstantsDeploymentPositions.h
        void RotateArm(double rotations);

        /// Drives the deployment arm to an angle relative to the current angle
        /// \param rotation  Percentage of max roation to apply [0, 1]
        void RotateArmRelative(double rotation);
        void RotateArmSetAbsPos(double absPos) { m_absPos = absPos; }
        double m_absPos = 0.0;

        /// Extends the deployment arm
        void ExtendArm();

        /// Retracts the deployment arm
        void RetractArm();

        /// Extends the back plate
        void ExtendBackPlate();

        /// Retracts the back plate
        void RetractBackPlate();

        /// Stops running the deployment arm motor
        void Stop();

        /// Checks to see if the deployment arm is at a specified degree setpoint
        /// \param setpoint intended setpoint to check current position against
        /// \returns if current position is within an accepted margin error of the setpoint 
        bool IsAtSetpoint(double setpoint);

        /// Zero out the arm encoder count
        void ResetEncoder() { m_enc.SetPosition(0.0); RotateArm(0.0); }

        /// Reset relative encoder based on the absolute encoder
        void ResetEncoderWithAbsolute();

        /// Converts absolute encoder value to relative encoder value
        double AbsoluteToRelative(double absPos);

    private:
        CANSparkMax m_motor;
        SparkMaxAlternateEncoder m_enc{m_motor.GetAlternateEncoder(SparkMaxAlternateEncoder::Type::kQuadrature, 4096)};
        SparkMaxPIDController m_pid{m_motor.GetPIDController()};
        frc::PIDController m_absPid{40.0, 0.0, 0.0};
        double m_setpointTicks = 0.0;
        frc::Solenoid m_armSolenoid;
        frc::Solenoid m_backPlateSolenoid;
        frc::Timer m_timer;

        bool m_resetingEncoder = false;

        // Empirically measured 4657 motor ticks for 140 degrees of arm rotation
        //static constexpr double kDegreesPerTick = 140.0 / 4657.0;
        // Empirically measured 5815 motor ticks for 180 degrees of arm rotation
        // static constexpr double kDegreesPerTick = 180.0 / (17.85 + 33.81);
        // Empirically measured multiple data points and made linear regression
        // Linear Regression Slope
        // static constexpr double kTicksPerDegree = 1.01;
        static constexpr double kTicksPerDegree = 1.0 / 2.07;
        static constexpr double kTickOffset = 0.0;
        // Initial tick position when the arm is in travel position
        static constexpr double kInitialPosition = 4.8 * kTicksPerDegree;
        static constexpr double kDegreesPerTick = 1.0 / kTicksPerDegree;
        static constexpr double kMaxOperatorDeg = 5.0;

        static constexpr bool kArmSolenoidExtend = true;
        static constexpr bool kArmSolenoidRetract = false;
        static constexpr bool kBackPlateSolenoidExtend = true;
        static constexpr bool kBackPlateSolenoidRetract = false;

        frc::DutyCycleEncoder m_absEnc;

        wpi::log::DoubleLogEntry m_logArmAngle;
        wpi::log::DoubleLogEntry m_logAbsEnc;
        wpi::log::DoubleLogEntry m_logOutputCurrent;
        wpi::log::DoubleLogEntry m_logMotorOutput;
        wpi::log::DoubleLogEntry m_logMotorTemp;
        wpi::log::DoubleLogEntry m_logFwdLimit;
        wpi::log::DoubleLogEntry m_logRevLimit;
};