
//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302 
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include <State.h>
#include <mechanisms/base/Mech1ServoState.h>
#include <mechanisms/base/Mech1Servo.h>
#include <utils/Logger.h>

// Third Party Includes

using namespace std;

/// @class Mech1ServoState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
Mech1ServoState::Mech1ServoState
(
    Mech1Servo*                     mechanism,
    std::string                     stateName,
    int                             stateId,
    double                          target
) : State(stateName, stateId),
    m_mechanism( mechanism ),
    m_target( target )
{
    if ( mechanism == nullptr )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("Mech1Servo"), ("Mech1ServoState::Mech1ServoState"), string("no mechanism"));
    }    
}

void Mech1ServoState::Init()
{
}


void Mech1ServoState::Run()           
{
    if (m_mechanism != nullptr)
    {
        m_mechanism->SetAngle(m_target);
    }
}

void Mech1ServoState::Exit() 
{
}

bool Mech1ServoState::AtTarget() const
{
    return true;
}

void Mech1ServoState::LogInformation() const
{
    if (m_mechanism != nullptr)
    {
        auto ntName = m_mechanism->GetNetworkTableName();
        auto id = GetStateName();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, string("state name"), id);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, string("Target"), GetTarget());
    }
}
