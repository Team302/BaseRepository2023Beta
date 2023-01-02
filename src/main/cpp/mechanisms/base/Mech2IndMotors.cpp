
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

// Team 302 includes
#include <mechanisms/base/Mech.h>
#include <mechanisms/base/Mech1IndMotor.h>
#include <mechanisms/base/Mech2IndMotors.h>
#include <mechanisms/controllers/ControlData.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <utils/Logger.h>

// Third Party Includes
#include <units/time.h>

using namespace std;

/// @brief Create a generic mechanism wiht 2 independent motors 
/// @param [in] MechanismTypes::MECHANISM_TYPE the type of mechansim
/// @param [in] std::string the name of the file that will set control parameters for this mechanism
/// @param [in] std::string the name of the network table for logging information
/// @param [in] std::shared_ptr<IDragonMotorController> primary motor used by this mechanism
/// @param [in] std::shared_ptr<IDragonMotorController> secondary motor used by this mechanism
Mech2IndMotors::Mech2IndMotors
(
    MechanismTypes::MECHANISM_TYPE              type,
    std::string                                 controlFileName,
    std::string                                 networkTableName,
    shared_ptr<IDragonMotorController>          primaryMotor,
    shared_ptr<IDragonMotorController>          secondaryMotor
) : Mech(type, controlFileName, networkTableName),
    m_primary( primaryMotor),
    m_secondary( secondaryMotor),
    m_primaryTarget(0.0),
    m_secondaryTarget(0.0)
{
    if ( primaryMotor.get() == nullptr )
    {
        Logger::GetLogger()->LogData( LOGGER_LEVEL::ERROR_ONCE, networkTableName, string( "Mech2IndMotors constructor" ), string( "failed to create primary control" ) );
    }    
    
    if ( secondaryMotor.get() == nullptr )
    {
        Logger::GetLogger()->LogData( LOGGER_LEVEL::ERROR_ONCE, networkTableName, string( "Mech2IndMotors constructor" ), string( "failed to create secondary control" ) );
    }
}



/// @brief update the output to the mechanism using the current controller and target value(s)
/// @return void 
void Mech2IndMotors::Update()
{
    if ( m_primary.get() != nullptr )
    {
        m_primary.get()->Set(m_primaryTarget);
    }
    if ( m_secondary.get() != nullptr )
    {
        m_secondary.get()->Set(m_secondaryTarget);
    }

    LogInformation();
}

void Mech2IndMotors::UpdateTargets
(
    double      primary,
    double      secondary

)
{
    m_primaryTarget = primary;
    m_secondaryTarget = secondary;
    Update();
}


/// @brief  Return the current position of the primary motor in the mechanism.  The value is in inches or degrees.
/// @return double	position in inches (translating mechanisms) or degrees (rotating mechanisms)
double Mech2IndMotors::GetPrimaryPosition() const 
{
    return ( m_primary.get() != nullptr ) ? m_primary.get()->GetRotations() * 360.0 : 0.0;
}

/// @brief  Return the current position of the secondary motor in the mechanism.  The value is in inches or degrees.
/// @return double	position in inches (translating mechanisms) or degrees (rotating mechanisms)
double Mech2IndMotors::GetSecondaryPosition() const 
{
    return ( m_secondary.get() != nullptr ) ? m_secondary.get()->GetRotations() * 360.0 : 0.0;
}

/// @brief  Get the current speed of the primary motor in the mechanism.  The value is in inches per second or degrees per second.
/// @return double	speed in inches/second (translating mechanisms) or degrees/second (rotating mechanisms)
double Mech2IndMotors::GetPrimarySpeed() const 
{
    return ( m_primary.get() != nullptr ) ? m_primary.get()->GetRPS() : 0.0;
}

/// @brief  Get the current speed of the secondary motor in the mechanism.  The value is in inches per second or degrees per second.
/// @return double	speed in inches/second (translating mechanisms) or degrees/second (rotating mechanisms)
double Mech2IndMotors::GetSecondarySpeed() const 
{
    return ( m_secondary.get() != nullptr ) ? m_secondary.get()->GetRPS() : 0.0;
}

/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] ControlData*                                   pid:  the control constants
/// @return void
void Mech2IndMotors::SetControlConstants
(
    int                                         slot,
    ControlData*                                pid                 
) 
{
    if ( m_primary.get() != nullptr )
    {
        m_primary.get()->SetControlConstants(slot, pid);
    }
}
void Mech2IndMotors::SetSecondaryControlConstants
(
    int                                         slot,
    ControlData*                                pid                 
) 
{
    if ( m_secondary.get() != nullptr )
    {
        m_secondary.get()->SetControlConstants(slot, pid);
    }    
}


/// @brief log data to the network table if it is activated and time period has past
void Mech2IndMotors::LogInformation() const
{
    auto ntName = GetNetworkTableName();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, "Speed - Primary", GetPrimarySpeed() );
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, "Speed - Secondary", GetSecondarySpeed() );
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, "Position - Primary", GetPrimaryPosition() );
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, "Position - Secondary", GetSecondaryPosition() );
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, "Target - Primary", m_primaryTarget);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, "Target - Secondary", m_secondaryTarget);
}

