
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

#pragma once

#include <string>

class DragonAssert
{
    public:

        /// @enum DRAGONASSERT_OPTION
        /// @brief Define how assert evaluations should be handled
        enum DRAGONASSERT_OPTION
        {
            PROCESS,        /// Assert if condition is false
            NO_OP           /// NO-OP - skip check
        };

        /// @brief Find or create the singleton DragonAssert
        /// @returns DragonAssert* pointer to the DragonAssert
        static DragonAssert* GetDragonAssert();

        /// @brief assert if condition is false.  In NO_OP mode, this will be skipped, but message will be logged
        /// @param [in] bool    condition to check
        /// @param [in] std::string: message/value
        void Assert
        (
            bool                condition,
            std::string         msg
        );

        /// @brief assert if condition is false.  In NO_OP mode, the condition will be returned instead of asserting & 
        /// @brief message will be logged if failed.  This is useful for if conditionals.
        /// @param [in] bool    condition to check
        /// @param [in] std::string: message/value
        /// @returns true if condition is true, otherwise asserts/gives message
        bool Always
        (
            bool                condition,
            std::string         msg
        );

        /// @brief assert if condition is true. In NO_OP mode, the condition will be returned instead of asserting & 
        /// @brief message will be logged if failed.  This is useful for if conditionals.
        /// @param [in] bool    condition to check
        /// @param [in] std::string: message/value
        /// @returns true if condition is true, otherwise asserts/gives message
        bool Never
        (
            bool                condition,
            std::string         msg
        );

    private:
        DRAGONASSERT_OPTION     m_option;


        DragonAssert();
        ~DragonAssert() = default;

        static DragonAssert*                          m_instance;
};


