
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

#include <mechanisms/base/IState.h>
#include <mechanisms/base/Mech.h>
#include <mechanisms/base/StateMgr.h>
#include <mechanisms/controllers/MechanismTargetData.h>
#include <mechanisms/MechanismFactory.h>
#include <mechanisms/MechanismTypes.h>
#include <mechanisms/StateMgrHelper.h>
#include <mechanisms/StateStruc.h>
#include <utils/Logger.h>

using namespace std;

void StateMgrHelper::RunCurrentMechanismStates() 
{
    for (auto i=MechanismTypes::MECHANISM_TYPE::UNKNOWN_MECHANISM+1; i<MechanismTypes::MECHANISM_TYPE::MAX_MECHANISM_TYPES; ++i)
    {
        auto mech = MechanismFactory::GetMechanismFactory()->GetMechanism(static_cast<MechanismTypes::MECHANISM_TYPE>(i));
        auto stateMgr = mech != nullptr ? mech->GetStateMgr() : nullptr;
        if (stateMgr != nullptr)
        {
            stateMgr->RunCurrentState();
        }
    }   
}

IState* StateMgrHelper::CreateState
(
    Mech*                       mech,
    StateStruc&                 stateInfo,
    MechanismTargetData*        targetData
)
{
    auto controlData = targetData->GetController();
    auto controlData2 = targetData->GetController2();
    auto target = targetData->GetTarget();
    auto secondaryTarget = targetData->GetSecondTarget();
    auto robotPitch = targetData->GetRobotPitch();
    auto function1Coeff = targetData->GetFunction1Coeff();
    auto function2Coeff = targetData->GetFunction2Coeff();
    auto type = stateInfo.type;
    IState* thisState = nullptr;
    switch (type)
    {
        // @ADDMECH Add case(s) tto create your state(s) 
        //case StateType::SHOOTER:
        //    thisState = new ShooterState(controlData, 
        //                                    controlData2, 
        //                                    target, 
        //                                    secondaryTarget);
        //    break;

        default:
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, mech->GetNetworkTableName(), string("StateMgr::StateMgr"), string("unknown state"));
            break;
    }
    return thisState;
}
