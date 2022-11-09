
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

// C++ includes
#include <string>
#include <cstring>

// wpilib includes

// team 302 includes
#include <hw/DragonCanCoder.h>
#include <hw/xml/CancoderXmlParser.h>
#include <utils/HardwareIDValidation.h>
#include <utils/Logger.h>

// third party includes
#include <pugixml/pugixml.hpp>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <ctre/phoenix/ErrorCode.h>

using namespace std;
using namespace pugi;

/// @brief parses the cancoder node in the robot.xml file and creates a cancoder
/// @param [in] xml_node - the cancoder element in the xml file
/// @return shared_ptr<CANCoder
DragonCanCoder* CancoderXmlParser::ParseXML
(
    string              networkTableName,
    xml_node            CanCoderNode
)
{
    DragonCanCoder* cancoder = nullptr;

    string usage;
    string canBusName("rio");
    int canID = 0;
    double offset = 0.0;
    bool reverse = false;
    
    bool hasError = false;

    for(xml_attribute attr = CanCoderNode.first_attribute(); attr &&!hasError; attr = attr.next_attribute() )
    {
        if ( strcmp( attr.name(), "usage") == 0)
        {
            usage = attr.value();
        }
        else if ( strcmp( attr.name(), "canId" ) == 0 )
        {
            canID = attr.as_int();
            hasError = HardwareIDValidation::ValidateCANID( canID, string( "CancoderXmlParser::ParseXML" ) );
        }        
        else if (strcmp(attr.name(), "canbus") == 0)
        {
            canBusName = attr.as_string();
        }
        else if ( strcmp( attr.name(), "offset" ) == 0 )
        {
            offset = attr.as_double();
        }        
        else if ( strcmp( attr.name(), "reverse" ) == 0 )
        {
            reverse = attr.as_bool();
        }        
        else
        {
            Logger::GetLogger()->LogData (LOGGER_LEVEL::ERROR_ONCE, string("CancoderXmlParser"), string("invalid attribute"), string(attr.value()));
            hasError = true;
        }

    }   
    if(!hasError)
    {
        cancoder = new DragonCanCoder(networkTableName, usage, canID, canBusName, offset, reverse);

    }
    return cancoder;
}