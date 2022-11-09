
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
   
#include <string>

#include <hw/DragonCanCoder.h>
#include <utils/Logger.h>

#include <ctre/phoenix/sensors/WPI_CANCoder.h>

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::sensors;

DragonCanCoder::DragonCanCoder
(
	string						networkTableName,
	string      			    usage,
	int 						canID,
    string                      canBusName,
    double                      offset,
    bool                        reverse
) : WPI_CANCoder(canID, canBusName),
	m_networkTableName(networkTableName),
    m_usage(usage)
{
    auto error = ConfigFactoryDefault(50);
    if ( error != ErrorCode::OKAY )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, networkTableName, string("ConfigFactoryDefault"), to_string(error));
    }
    error = ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180, 0);
    if ( error != ErrorCode::OKAY )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, networkTableName, string("ConfigAbsoluteSensorRange"), to_string(error));
    }

    error = ConfigMagnetOffset(offset, 0); 
    if ( error != ErrorCode::OKAY )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, networkTableName, string("ConfigMagnetOffset"), to_string(error));
    }

    error = ConfigSensorDirection(reverse, 0); 
    if ( error != ErrorCode::OKAY )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, networkTableName, string("ConfigSensorDirection"), to_string(error));
    }

    error = ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition, 0); 
    if ( error != ErrorCode::OKAY )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, networkTableName, string("ConfigSensorDirection"), to_string(error));
    }

    error = ConfigVelocityMeasurementPeriod(SensorVelocityMeasPeriod::Period_1Ms, 0);
    if ( error != ErrorCode::OKAY )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, networkTableName, string("ConfigVelocityMeasurementPeriod"), to_string(error));
    }

    error = ConfigVelocityMeasurementWindow(64, 0);
    if ( error != ErrorCode::OKAY )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, networkTableName, string("ConfigVelocityMeasurementWindow"), to_string(error));
    }
}
