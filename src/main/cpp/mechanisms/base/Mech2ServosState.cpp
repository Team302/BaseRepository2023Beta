
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

// FRC includes

// Team 302 includes
#include <mechanisms/base/IState.h>
#include <mechanisms/base/Mech2ServosState.h>
#include <mechanisms/base/Mech2Servos.h>
#include <utils/Logger.h>

// Third Party Includes

using namespace std;

/// @class Mech2ServosState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
Mech2ServosState::Mech2ServosState
(
    Mech2Servos*                    mechanism,
    double                          target,
    double                          target2
) : IState(),
    m_mechanism(mechanism),
    m_target(target),
    m_target2(target2)
{
    if ( mechanism == nullptr )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, mechanism->GetNetworkTableName(), ("Mech2ServosState::Mech2ServosState"), string("no mechanism"));
    }    
}

void Mech2ServosState::Init()
{
}


void Mech2ServosState::Run()           
{
    if (m_mechanism != nullptr)
    {
        m_mechanism->SetAngle(m_target);
        m_mechanism->SetAngle(m_target2);
        auto ntName = m_mechanism->GetNetworkTableName();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, string("Target"), GetTarget());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, string("Target2"), GetTarget2());
    }
}

void Mech2ServosState::Exit() 
{
}

bool Mech2ServosState::AtTarget() const
{
    return true;
}

