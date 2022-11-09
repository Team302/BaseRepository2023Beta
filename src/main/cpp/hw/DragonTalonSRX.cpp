
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302 
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes
#include <memory>
#include <string>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/PowerDistribution.h>
#include <frc/motorcontrol/MotorController.h>

// Team 302 includes
#include <hw/ctreadapters/DragonControlToCTREAdapter.h>
#include <hw/factories/DragonControlToCTREAdapterFactory.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/DragonTalonSRX.h>
#include <hw/factories/PDPFactory.h>
#include <hw/usages/MotorControllerUsage.h>
#include <hw/DistanceAngleCalcStruc.h>
#include <utils/ConversionUtils.h>
#include <utils/Logger.h>

// Third Party Includes
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>
#include <ctre/phoenix/motorcontrol/LimitSwitchType.h>


using namespace frc;
using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;


DragonTalonSRX::DragonTalonSRX
(
	string											networkTableName,
	MotorControllerUsage::MOTOR_CONTROLLER_USAGE	deviceType, 
	int 											deviceID, 
    int 											pdpID, 
	DistanceAngleCalcStruc							calcStruc,
	MOTOR_TYPE										motorType
) : m_networkTableName(networkTableName),
	m_talon( make_shared<WPI_TalonSRX>(deviceID)),
	m_controller(),
	m_type(deviceType),
	m_id(deviceID),
	m_pdp( pdpID ),
	m_calcStruc(calcStruc),
	m_motorType(motorType)
{
	m_networkTableName += string(" - motor ");
	m_networkTableName += to_string(deviceID);

	m_controller[0] = DragonControlToCTREAdapterFactory::GetFactory()->CreatePercentOuptutAdapter(networkTableName, m_talon.get());
	m_controller[0]->InitializeDefaults();
	for (auto i=1; i<4; ++i)
	{
		m_controller[i] = m_controller[0];
	}
	auto prompt = string("CTRE CAN motor controller ");
	prompt += to_string(deviceID);

	SupplyCurrentLimitConfiguration climit;
	climit.enable = false;
	climit.currentLimit = 1.0;
	climit.triggerThresholdCurrent = 1.0;
	climit.triggerThresholdTime = 0.001;
	auto error = m_talon.get()->ConfigSupplyCurrentLimit(climit, 50);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigSupplyCurrentLimit"), string("error"));
		error = ErrorCode::OKAY;
	}

	error = m_talon.get()->ConfigVoltageCompSaturation(12.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigVoltageCompSaturation"), string("error"));
		error = ErrorCode::OKAY;
	}

	error = m_talon.get()->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_Deactivated, LimitSwitchNormal::LimitSwitchNormal_Disabled, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigForwardLimitSwitchSource"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_Deactivated, LimitSwitchNormal::LimitSwitchNormal_Disabled, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigReverseLimitSwitchSource"), string("error"));
		error = ErrorCode::OKAY;
	}

	error = m_talon.get()->ConfigForwardSoftLimitEnable(false, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigForwardSoftLimitEnable"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigForwardSoftLimitThreshold(0.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigForwardSoftLimitThreshold"), string("error"));
		error = ErrorCode::OKAY;
	}

	error = m_talon.get()->ConfigReverseSoftLimitEnable(false, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigReverseSoftLimitEnable"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigReverseSoftLimitThreshold(0.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigReverseSoftLimitThreshold"), string("error"));
		error = ErrorCode::OKAY;
	}
	

	error = m_talon.get()->ConfigMotionSCurveStrength(0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigMotionSCurveStrength"), string("error"));
		error = ErrorCode::OKAY;
	}

	error = m_talon.get()->ConfigMotionProfileTrajectoryPeriod(0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigMotionProfileTrajectoryPeriod"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigMotionProfileTrajectoryInterpolationEnable(true, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigMotionProfileTrajectoryInterpolationEnable"), string("error"));
		error = ErrorCode::OKAY;
	}

	m_talon.get()->ConfigAllowableClosedloopError(0.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigAllowableClosedloopError"), string("error"));
		error = ErrorCode::OKAY;
	}

	error = m_talon.get()->ConfigRemoteFeedbackFilter(60, RemoteSensorSource::RemoteSensorSource_Off, 0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigRemoteFeedbackFilter"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigRemoteFeedbackFilter(60, RemoteSensorSource::RemoteSensorSource_Off, 1, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigRemoteFeedbackFilter"), string("error"));
	}
}

double DragonTalonSRX::GetRotations() const
{
	if (m_calcStruc.countsPerDegree > 0.01)
	{
		return m_talon.get()->GetSelectedSensorPosition() / (m_calcStruc.countsPerDegree * 360.0);
	}
	return (ConversionUtils::CountsToRevolutions( (m_talon.get()->GetSelectedSensorPosition()), m_calcStruc.countsPerRev) / m_calcStruc.gearRatio);
}

double DragonTalonSRX::GetRPS() const
{
	if (m_calcStruc.countsPerDegree > 0.01)
	{
		return m_talon.get()->GetSelectedSensorVelocity() * 10.0 / (m_calcStruc.countsPerDegree * 360.0);
	}
	return (ConversionUtils::CountsPer100msToRPS( m_talon.get()->GetSelectedSensorVelocity(), m_calcStruc.countsPerRev) / m_calcStruc.gearRatio);
}

shared_ptr<MotorController> DragonTalonSRX::GetSpeedController() const
{
	return m_talon;
}

double DragonTalonSRX::GetCurrent() const
{
	auto pdp = PDPFactory::GetFactory()->GetPDP();
	if (pdp != nullptr)
	{
		return pdp->GetCurrent(m_pdp);
	}
	return 0.0;
}

void DragonTalonSRX::UpdateFramePeriods
(
	ctre::phoenix::motorcontrol::StatusFrameEnhanced	frame,
	uint8_t												milliseconds
)
{
	m_talon.get()->SetStatusFramePeriod( frame, milliseconds, 0 );
}
void DragonTalonSRX::SetFramePeriodPriority
(
	MOTOR_PRIORITY              priority
)
{
	switch ( priority )
	{
		case HIGH:
			UpdateFramePeriods( StatusFrameEnhanced::Status_1_General, 10 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_2_Feedback0, 20 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_3_Quadrature, 100 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_4_AinTempVbat, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_8_PulseWidth, 120 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_10_Targets, 120 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_11_UartGadgeteer, 120 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_12_Feedback1, 120 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_13_Base_PIDF0, 120 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_14_Turn_PIDF1, 120 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_15_FirmareApiStatus, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_Brushless_Current, 200 );
			break;

		case MEDIUM:
			UpdateFramePeriods( StatusFrameEnhanced::Status_1_General, 20 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_2_Feedback0, 30 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_3_Quadrature, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_4_AinTempVbat, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_8_PulseWidth, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_10_Targets, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_11_UartGadgeteer, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_12_Feedback1, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_13_Base_PIDF0, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_14_Turn_PIDF1, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_15_FirmareApiStatus, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_Brushless_Current, 200 );
			break;

		case LOW:
			UpdateFramePeriods( StatusFrameEnhanced::Status_1_General, 30 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_2_Feedback0, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_3_Quadrature, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_4_AinTempVbat, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_8_PulseWidth, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_10_Targets, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_11_UartGadgeteer, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_12_Feedback1, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_13_Base_PIDF0, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_14_Turn_PIDF1, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_15_FirmareApiStatus, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_Brushless_Current, 200 );
			break;

		default:
		break;

	}
}



void DragonTalonSRX::Set(double value)
{
	m_controller[0]->Set(value);
}
void DragonTalonSRX::SetRotationOffset(double rotations)
{
//	double newRotations = -rotations + DragonTalonSRX::GetRotations();
//	m_tickOffset += (int) (newRotations * m_calcStruc.countsPerRev / m_calcStruc.gearRatio);
}

void DragonTalonSRX::SetVoltageRamping(double ramping, double rampingClosedLoop)
{
    m_talon.get()->ConfigOpenloopRamp(ramping);

    if (rampingClosedLoop >= 0)
    {
  	m_talon.get()->ConfigClosedloopRamp(rampingClosedLoop);
    }
}


void DragonTalonSRX::EnableCurrentLimiting(bool enabled)
{
    m_talon.get()->EnableCurrentLimit(enabled);
}

void DragonTalonSRX::EnableBrakeMode(bool enabled)
{
    m_talon.get()->SetNeutralMode(enabled ? ctre::phoenix::motorcontrol::NeutralMode::Brake : ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void DragonTalonSRX::Invert(bool inverted)
{
    m_talon.get()->SetInverted(inverted);
}

void DragonTalonSRX::SetSensorInverted(bool inverted)
{
    m_talon.get()->SetSensorPhase(inverted);
}

MotorControllerUsage::MOTOR_CONTROLLER_USAGE DragonTalonSRX::GetType() const
{
	return m_type;
}

int DragonTalonSRX::GetID() const
{
	return m_id;
}

//------------------------------------------------------------------------------
// Method:		SelectClosedLoopProfile
// Description:	Selects which profile slot to use for closed-loop control
// Returns:		void
//------------------------------------------------------------------------------
void DragonTalonSRX::SelectClosedLoopProfile
(
	int	   slot,			// <I> - profile slot to select
	int    pidIndex			// <I> - 0 for primary closed loop, 1 for cascaded closed-loop
)
{
	auto error = m_talon.get()->SelectProfileSlot( slot, pidIndex );
	if ( error != ErrorCode::OKAY )
	{
		auto prompt = string("Dragon Talon");
		prompt += to_string(m_talon.get()->GetDeviceID());
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE,prompt, string("SelectProfileSlot"), string("error"));
	}
}


int DragonTalonSRX::ConfigSelectedFeedbackSensor
(
	FeedbackDevice feedbackDevice,
	int pidIdx,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		error = m_talon.get()->ConfigSelectedFeedbackSensor( feedbackDevice, pidIdx, timeoutMs );
	}
	return error;
}

int DragonTalonSRX::ConfigSelectedFeedbackSensor
(
	RemoteFeedbackDevice feedbackDevice,
	int pidIdx,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		error = m_talon.get()->ConfigSelectedFeedbackSensor( feedbackDevice, pidIdx, timeoutMs );
	}
	return error;
}

int DragonTalonSRX::ConfigPeakCurrentLimit
(
	int amps,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		error = m_talon.get()->ConfigPeakCurrentLimit( amps, timeoutMs );
	}
	return error;
}

int DragonTalonSRX::ConfigPeakCurrentDuration
(
	int milliseconds,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		error = m_talon.get()->ConfigPeakCurrentDuration( milliseconds, timeoutMs );
	}
	return error;
}

int DragonTalonSRX::ConfigContinuousCurrentLimit
(
	int amps,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		error = m_talon.get()->ConfigContinuousCurrentLimit( amps, timeoutMs );
	}
	return error;
}

void DragonTalonSRX::SetAsFollowerMotor
(
    int         masterCANID         // <I> - master motor
)
{
    m_talon.get()->Set( ControlMode::Follower, masterCANID );
}


/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] int             slot - hardware slot to use
/// @param [in] ControlData*    pid - the control constants
/// @return void
void DragonTalonSRX::SetControlConstants(int slot, ControlData* controlInfo)
{
	delete m_controller[slot];
	m_controller[slot] = DragonControlToCTREAdapterFactory::GetFactory()->CreateAdapter(m_networkTableName, slot, controlInfo, m_calcStruc, m_talon.get());
}

void DragonTalonSRX::SetForwardLimitSwitch
( 
	bool normallyOpen
)
{
	LimitSwitchNormal type = normallyOpen ? LimitSwitchNormal::LimitSwitchNormal_NormallyOpen : LimitSwitchNormal::LimitSwitchNormal_NormallyClosed;
	m_talon.get()->ConfigForwardLimitSwitchSource( LimitSwitchSource::LimitSwitchSource_FeedbackConnector, type, 0  );
	m_talon.get()->OverrideLimitSwitchesEnable(true);
}

void DragonTalonSRX::SetReverseLimitSwitch
(
	bool normallyOpen
)
{
	LimitSwitchNormal type = normallyOpen ? LimitSwitchNormal::LimitSwitchNormal_NormallyOpen : LimitSwitchNormal::LimitSwitchNormal_NormallyClosed;
	m_talon.get()->ConfigReverseLimitSwitchSource( LimitSwitchSource::LimitSwitchSource_FeedbackConnector, type, 0  );
	m_talon.get()->OverrideLimitSwitchesEnable(true);
}


void DragonTalonSRX::SetRemoteSensor
(
    int                                             canID,
    ctre::phoenix::motorcontrol::RemoteSensorSource deviceType
)
{
	m_talon.get()->ConfigRemoteFeedbackFilter( canID, deviceType, 0, 0.0 );
	m_talon.get()->ConfigSelectedFeedbackSensor( RemoteFeedbackDevice::RemoteFeedbackDevice_RemoteSensor0, 0, 0 );
}

void DragonTalonSRX::SetDiameter
(
	double 	diameter
)
{
	m_calcStruc.diameter = diameter;
}

void DragonTalonSRX::SetVoltage
(
	units::volt_t output
)
{
	m_talon.get()->SetVoltage(output);
}


 bool DragonTalonSRX::IsForwardLimitSwitchClosed() const
 {
	auto sensors = m_talon.get()->GetSensorCollection();
	auto closed = sensors.IsFwdLimitSwitchClosed();
	return closed == 1;
}

bool DragonTalonSRX::IsReverseLimitSwitchClosed() const
{
	auto sensors = m_talon.get()->GetSensorCollection();
	auto closed = sensors.IsRevLimitSwitchClosed();
	return closed == 1;
}

void DragonTalonSRX::EnableVoltageCompensation( double fullvoltage) 
{
	m_talon.get()->ConfigVoltageCompSaturation(fullvoltage);
	m_talon.get()->EnableVoltageCompensation(true);
}

void DragonTalonSRX::SetSelectedSensorPosition
(
	double  initialPosition
) 
{
	m_talon.get()->SetSelectedSensorPosition(initialPosition, 0, 50);
}
        
double DragonTalonSRX::GetCountsPerInch() const 
{
	return m_calcStruc.countsPerInch;
}
double DragonTalonSRX::GetCountsPerDegree() const 
{
	return m_calcStruc.countsPerDegree;
}

/**
ControlModes::CONTROL_TYPE DragonTalonSRX::GetControlMode() const
{
	return m_controlMode;
}
**/

double DragonTalonSRX::GetCounts() const 
{
	return m_talon.get()->GetSelectedSensorPosition();
}

IDragonMotorController::MOTOR_TYPE DragonTalonSRX::GetMotorType() const
{
	return m_motorType;
}

void DragonTalonSRX::EnableDisableLimitSwitches
(
	bool enable
)
{
	m_talon.get()->OverrideLimitSwitchesEnable(enable);
}