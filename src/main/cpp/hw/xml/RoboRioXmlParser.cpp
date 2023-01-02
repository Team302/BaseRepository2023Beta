
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

// FRC includes
#include <frc/BuiltInAccelerometer.h>

// Team 302 includes
#include <hw/factories/DragonRoboRioAccelerometerFactory.h>
#include <hw/factories/DragonServoFactory.h>
#include <hw/usages/ServoUsage.h>
#include <hw/xml/RoboRioXmlParser.h>
#include <utils/HardwareIDValidation.h>
#include <utils/Logger.h>

// Third Party Includes
#include <pugixml/pugixml.hpp>

using namespace std;

/// @brief Parse a roborio XML element and create a DragonRoboRioAccelerometer from its definition.
void RoboRioXmlParser::ParseXML
(
    pugi::xml_node      roboRioNode
)
{
    // initialize attributes to default values
    auto orient = RoboRioOrientation::ROBORIO_ORIENTATION:: X_FORWARD_Y_LEFT;
    auto hasError = false;

    // parse/validate the xml
    for (pugi::xml_attribute attr = roboRioNode.first_attribute(); attr && !hasError; attr = attr.next_attribute())
    {
        string attrName (attr.name());
        if (attrName.compare("orientation") == 0)
        {
            auto it = RoboRioOrientation::GetInstance()->ROBORIO_ORIENTATION_MAP.find(attr.value());
            if (it != RoboRioOrientation::GetInstance()->ROBORIO_ORIENTATION_MAP.end())
            {
                orient = it->second;
            }
            else
            {
                string msg = "Invalid Orientation Option ";
                msg += attr.value();
                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("RoboRioXmlParser"), string("ParseXML"), msg );
                hasError = true;
            }
        }
        else
        {
            string msg = "unknown attribute ";
            msg += attr.name();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("RoboRioXmlParser"), string("ParseXML"), msg );
            hasError = true;
        }
    }

    // create the object
    if ( !hasError )
    {
        DragonRoboRioAccelerometerFactory::GetInstance()->CreateAccelerometer(orient);
    }
}
