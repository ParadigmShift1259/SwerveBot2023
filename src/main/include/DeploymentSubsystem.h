
#pragma once

#include <wpi/DataLog.h>

#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <frc/DataLogManager.h>
#include <frc/DutyCycleEncoder.h>
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

        /// Stops running the deployment arm motor
        void Stop();

        /// Checks to see if the deployment arm is at a specified degree setpoint
        /// \param setpoint intended setpoint to check current position against
        /// \returns if current position is within an accepted margin error of the setpoint 
        bool IsAtSetpoint(double setpoint);

        /// Checks to see if the deployment arm is at a specified degree setpoint using the absolute encoder
        /// \param setpoint intended setpoint to check current position against
        /// \returns if current position is within an accepted margin error of the setpoint 
        bool IsAtAbsoluteSetpoint(double setpoint);

        /// Set the arm encoder count to the specified position
        void ResetEncoder(double position) { m_enc.SetPosition(position); }

        /// Reset relative encoder based on the absolute encoder
        void ResetEncoderWithAbsolute();

        /// Converts absolute encoder value to relative encoder value
        /// Only valid if arm is close to travel position
        double AbsoluteToRelative(double absPos);

    private:
        CANSparkMax m_motor;
        SparkMaxAlternateEncoder m_enc{m_motor.GetAlternateEncoder(SparkMaxAlternateEncoder::Type::kQuadrature, 4096)};
        SparkMaxPIDController m_pid{m_motor.GetPIDController()};
        double m_setpointTicks = 0.0;
        frc::Solenoid m_armSolenoid;
        frc::Timer m_timer;

        bool m_resetingEncoder = false;

        static constexpr bool kArmSolenoidExtend = true;
        static constexpr bool kArmSolenoidRetract = false;

        frc::DutyCycleEncoder m_absEnc;
        SparkMaxRelativeEncoder m_neoEnc = m_motor.GetEncoder();

        // wpi::log::DoubleLogEntry m_logArmEnc;
        // wpi::log::DoubleLogEntry m_logAbsEnc;
        // wpi::log::DoubleLogEntry m_logNeoEnc;
        wpi::log::DoubleArrayLogEntry m_logEncVal;
        wpi::log::DoubleLogEntry m_logOutputCurrent;
        wpi::log::DoubleLogEntry m_logMotorOutput;
        wpi::log::DoubleLogEntry m_logMotorTemp;
        wpi::log::DoubleLogEntry m_logFwdLimit;
        wpi::log::DoubleLogEntry m_logRevLimit;
};