
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
#include <string>

// FRC includes

// Team 302 includes
#include <hw/DistanceAngleCalcStruc.h>
#include <hw/ctreadapters/DragonControlToCTREAdapter.h>
#include <hw/factories/DragonControlToCTREAdapterFactory.h>
#include <mechanisms/controllers/ControlData.h>
#include <utils/Logger.h>

#include <hw/ctreadapters/DragonControlToCTREAdapter.h>
#include <hw/ctreadapters/DragonPercentOutputToCTREAdapter.h>
#include <hw/ctreadapters/DragonPositionDegreeToCTREAdapter.h>
#include <hw/ctreadapters/DragonPositionInchToCTREAdapter.h>
#include <hw/ctreadapters/DragonTrapezoidToCTREAdapter.h>
#include <hw/ctreadapters/DragonVelocityDegreeToCTREAdapter.h>
#include <hw/ctreadapters/DragonVelocityInchToCTREAdapter.h>
#include <hw/ctreadapters/DragonVelocityRPSToCTREAdapter.h>
#include <hw/ctreadapters/DragonVoltageToCTREAdapter.h>



// Third Party Includes
#include <ctre/phoenix/motorcontrol/can/WPI_BaseMotorController.h>



using namespace std;


DragonControlToCTREAdapterFactory* DragonControlToCTREAdapterFactory::m_factory = nullptr;



//=======================================================================================
/// Method: GetInstance
/// @brief  Get the factory singleton
/// @return DragonServoFactory*    pointer to the factory
//=======================================================================================
DragonControlToCTREAdapterFactory* DragonControlToCTREAdapterFactory::GetFactory()
{
    if ( DragonControlToCTREAdapterFactory::m_factory == nullptr )
    {
        DragonControlToCTREAdapterFactory::m_factory = new DragonControlToCTREAdapterFactory();
    }
    return DragonControlToCTREAdapterFactory::m_factory;
}


DragonControlToCTREAdapter* DragonControlToCTREAdapterFactory::CreatePercentOuptutAdapter
(
    std::string                                                     networkTableName,  
    ctre::phoenix::motorcontrol::can::WPI_BaseMotorController*      controller
)
{    

    ControlData controlInfo;
    DistanceAngleCalcStruc calcStruc;
    return CreateAdapter(networkTableName, 0, &controlInfo, calcStruc, controller);
}
DragonControlToCTREAdapter* DragonControlToCTREAdapterFactory::CreateAdapter
(
    std::string                                                     networkTableName,  
    int                                                             controllerSlot, 
    ControlData*                                                    controlInfo,          
    DistanceAngleCalcStruc                                          calcStruc,                          
    ctre::phoenix::motorcontrol::can::WPI_BaseMotorController*      controller
)
{
    if (controlInfo != nullptr && controller != nullptr)
    {
        switch (controlInfo->GetMode())
        {
            case ControlModes::CONTROL_TYPE::PERCENT_OUTPUT:
                return new DragonPercentOutputToCTREAdapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
                break;

			case ControlModes::CONTROL_TYPE::POSITION_ABSOLUTE:
                break;

			case ControlModes::CONTROL_TYPE::POSITION_DEGREES:
                return new DragonPositionDegreeToCTREAdapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
                break;

			case ControlModes::CONTROL_TYPE::POSITION_INCH:
                return new DragonPositionInchToCTREAdapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
                break;

			case ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE:
                return new DragonPercentOutputToCTREAdapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
                break;

			case ControlModes::CONTROL_TYPE::TRAPEZOID:		
                return new DragonTrapezoidToCTREAdapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
                break;

			case ControlModes::CONTROL_TYPE::VELOCITY_DEGREES:
                return new DragonVelocityDegreeToCTREAdapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
                break;

			case ControlModes::CONTROL_TYPE::VELOCITY_INCH:
                return new DragonVelocityInchToCTREAdapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
                break;

			case ControlModes::CONTROL_TYPE::VELOCITY_RPS:
                return new DragonVelocityRPSToCTREAdapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
                break;

			case ControlModes::CONTROL_TYPE::CURRENT:
                break;

			case ControlModes::CONTROL_TYPE::MOTION_PROFILE:
                break;

			case ControlModes::CONTROL_TYPE::MOTION_PROFILE_ARC:
                break;

            case ControlModes::CONTROL_TYPE::VOLTAGE:
                return new DragonVoltageToCTREAdapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
                break;

			default:
                string msg{"Invalid contrrol data "};
                msg += to_string(controller->GetDeviceID());
				Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonControlToCTREAdapterFactory"), string("CreateAdapter"), msg);
                return new DragonPercentOutputToCTREAdapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
                break;
		}	
    }
    string msg{"Invalid contrrol information "};
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonControlToCTREAdapterFactory"), string("CreateAdapter"), msg);
    return new DragonPercentOutputToCTREAdapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
}




