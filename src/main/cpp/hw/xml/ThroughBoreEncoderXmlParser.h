#pragma once

#include <string>
#include <pugixml/pugixml.hpp>

namespace frc
{
    class Encoder;
}

class ThroughBoreEncoderXmlParser
{
    public:
        ThroughBoreEncoderXmlParser() = default;

        virtual ~ThroughBoreEncoderXmlParser() = default;
        frc::Encoder* ParseXML
        (
            std::string         networkTableName,
            pugi::xml_node throughBoreEncoderNode
        );

};