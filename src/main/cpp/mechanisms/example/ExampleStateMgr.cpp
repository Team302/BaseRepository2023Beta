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
#include <map>

// FRC includes

// Team 302 includes
#include <TeleopControl.h>
#include <auton/PrimitiveParams.h>
#include <mechanisms/MechanismFactory.h>
#include <mechanisms/base/StateMgr.h>
#include <mechanisms/StateStruc.h>
#include <mechanisms/example/Example.h>
#include <mechanisms/example/ExampleState.h>
#include <mechanisms/example/ExampleStateMgr.h>

// Third Party Includes

using namespace std;


ExampleStateMgr* ExampleStateMgr::m_instance = nullptr;
ExampleStateMgr* ExampleStateMgr::GetInstance()
{
	if ( ExampleStateMgr::m_instance == nullptr )
	{
        auto example = MechanismFactory::GetMechanismFactory()->GetExample();
        if (example != nullptr)
        {
            ExampleStateMgr::m_instance = new ExampleStateMgr();
        }
	}
	return ExampleStateMgr::m_instance;
    
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
ExampleStateMgr::ExampleStateMgr() : StateMgr(),
                                     m_example(MechanismFactory::GetMechanismFactory()->GetExample())
{
    map<string, StateStruc> stateMap;
    stateMap[m_exampleOffXmlString] = m_offState;
    stateMap[m_exampleForwardXmlString] = m_forwardState;
    stateMap[m_exampleReverseXmlString] = m_reverseState;  

    Init(m_example, stateMap);
    if (m_example != nullptr)
    {
        m_example->AddStateMgr(this);
    }
}   

/// @brief Check if driver inputs or sensors trigger a state transition
void ExampleStateMgr::CheckForStateTransition()
{

    if ( m_example != nullptr )
    {    
        auto currentState = static_cast<EXAMPLE_STATE>(GetCurrentState());
        auto targetState = currentState;

        auto controller = TeleopControl::GetInstance();
        auto isForwardSelected   = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::EXAMPLE_FORWARD) : false;
        auto isReverseSelected   = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::EXAMPLE_REVERSE) : false;

        if (isForwardSelected)
        {
            targetState = EXAMPLE_STATE::FORWARD;
        }
        else if (isReverseSelected)
        {
            targetState = EXAMPLE_STATE::REVERSE;
        }
        else
        {
            targetState = EXAMPLE_STATE::OFF;
        }

        if (targetState != currentState)
        {
            SetCurrentState(targetState, true);
        }
        
    }
}

/// @brief  Get the current Parameter parm value for the state of this mechanism
/// @param PrimitiveParams* currentParams current set of primitive parameters
/// @returns int state id - -1 indicates that there is not a state to set
int ExampleStateMgr::GetCurrentStateParam
(
    PrimitiveParams*    currentParams
) 
{
    // normally get the state from primitive params
    return StateMgr::GetCurrentStateParam(currentParams);
}