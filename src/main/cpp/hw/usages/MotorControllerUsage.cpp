
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
#include <map>
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include <hw/usages/MotorControllerUsage.h>
#include <utils/Logger.h>

// Third Party Includes

using namespace std;

MotorControllerUsage* MotorControllerUsage::m_instance = nullptr;
MotorControllerUsage* MotorControllerUsage::GetInstance()
{
    if ( m_instance == nullptr )
    {
        m_instance = new MotorControllerUsage();
    }
    return m_instance;
}

MotorControllerUsage::MotorControllerUsage()
{
    m_usageMap["SWERVE_DRIVE"]  = MOTOR_CONTROLLER_USAGE::SWERVE_DRIVE;
    m_usageMap["SWERVE_TURN"]   = MOTOR_CONTROLLER_USAGE::SWERVE_TURN;

    m_usageMap["DIFFERENTIAL_LEFT_MAIN"]  = MOTOR_CONTROLLER_USAGE::DIFFERENTIAL_LEFT_MAIN;
    m_usageMap["DIFFERENTIAL_LEFT_FOLLOWER"]   = MOTOR_CONTROLLER_USAGE::DIFFERENTIAL_LEFT_FOLLOWER;
    m_usageMap["DIFFERENTIAL_RIGHT_MAIN"]  = MOTOR_CONTROLLER_USAGE::DIFFERENTIAL_RIGHT_MAIN;
    m_usageMap["DIFFERENTIAL_RIGHT_FOLLOWER"]   = MOTOR_CONTROLLER_USAGE::DIFFERENTIAL_RIGHT_FOLLOWER;
    m_usageMap["MECANUM_LEFT_FRONT"] = MOTOR_CONTROLLER_USAGE::MECANUM_LEFT_FRONT;
    m_usageMap["MECANUM_LEFT_BACK"] = MOTOR_CONTROLLER_USAGE::MECANUM_LEFT_BACK;
    m_usageMap["MECANUM_RIGHT_FRONT"] = MOTOR_CONTROLLER_USAGE::MECANUM_RIGHT_FRONT;
    m_usageMap["MECANUM_RIGHT_BACK"] = MOTOR_CONTROLLER_USAGE::MECANUM_RIGHT_BACK;

    m_usageMap["EXAMPLE"]   = MOTOR_CONTROLLER_USAGE::EXAMPLE;
    // @ADDMECH add motors usages here
}

MotorControllerUsage::~MotorControllerUsage()
{
    m_usageMap.clear();
}

MotorControllerUsage::MOTOR_CONTROLLER_USAGE MotorControllerUsage::GetUsage
(
    string              usageString
)
{
    auto it = m_usageMap.find(usageString);
    if (it != m_usageMap.end())
    {
        return it->second;
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("MotorControllerUsage::GetUsage"), string("unknown usage"), usageString);
    return MotorControllerUsage::MOTOR_CONTROLLER_USAGE::UNKNOWN_MOTOR_CONTROLLER_USAGE;
}
