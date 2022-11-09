
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
#include <assert.h>

// FRC includes
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <utils/DragonAssert.h>
#include <utils/Logger.h>


// Third Party Includes

using namespace frc;
using namespace std;


/// @brief Find or create the singleton DragonAssert
/// @returns DragonAssert* pointer to the DragonAssert
DragonAssert* DragonAssert::m_instance = nullptr;
DragonAssert* DragonAssert::GetDragonAssert()
{
    if ( DragonAssert::m_instance == nullptr )
    {
        DragonAssert::m_instance = new DragonAssert();
    }
    return DragonAssert::m_instance;
}


/// @brief assert if condition is false
/// @param [in] bool    condition to check
/// @param [in] std::string: message/value
void DragonAssert::Assert
(
    bool                condition,
    std::string         msg
)
{
    if (m_option == DragonAssert::DRAGONASSERT_OPTION::PROCESS && !condition)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Assert"), msg, string("failed"));
        assert(condition);
    }
}

/// @brief assert if condition is false.  In NO_OP mode, the condition will be returned instead of asserting & 
/// @brief message will be logged if failed.  This is useful for if conditionals.
/// @param [in] bool    condition to check
/// @param [in] std::string: message/value
/// @returns true if condition is true, otherwise asserts/gives message
bool DragonAssert::Always
(
    bool                condition,
    std::string         msg
)
{
    if (!condition)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Assert"), msg, string("Always failed"));
        if (m_option == DragonAssert::DRAGONASSERT_OPTION::PROCESS)
        {
            assert(condition);
        }
    }
    return condition;
}

/// @brief assert if condition is true. In NO_OP mode, the condition will be returned instead of asserting & 
/// @brief message will be logged if failed.  This is useful for if conditionals.
/// @param [in] bool    condition to check
/// @param [in] std::string: message/value
/// @returns true if condition is false, otherwise asserts/gives message
bool DragonAssert::Never
(
    bool                condition,
    std::string         msg
)
{
    if (condition)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Assert"), msg, string("Never failed"));
        if (m_option == DragonAssert::DRAGONASSERT_OPTION::PROCESS)
        {
            assert(!condition);
        }
    }    
    return !condition;
}


DragonAssert::DragonAssert() : m_option(DRAGONASSERT_OPTION::NO_OP) 
{
}
