
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

// FRC includes

// Team 302 includes

// Third Party Includes
#include <string>
#include <frc/GenericHID.h>
#include <gamepad/IDragonGamePad.h>
#include <gamepad/DragonXBox.h>
#include <gamepad/DragonGamePad.h>
#include <TeleopControl.h>
#include <frc/DriverStation.h>
#include <utils/Logger.h>

using namespace frc;
using namespace std;

//----------------------------------------------------------------------------------
// Method:      GetInstance
// Description: If there isn't an instance of this class, it will create one.  The
//              single class instance will be returned.
// Returns:     OperatorInterface*  instance of this class
//----------------------------------------------------------------------------------
TeleopControl* TeleopControl::m_instance = nullptr; // initialize the instance variable to nullptr
TeleopControl* TeleopControl::GetInstance()
{
    if ( TeleopControl::m_instance == nullptr )
    {
        TeleopControl::m_instance = new TeleopControl();
    }
	if (TeleopControl::m_instance != nullptr && !TeleopControl::m_instance->IsInitialized())
	{
		TeleopControl::m_instance->Initialize();
	}
    return TeleopControl::m_instance;
}
//----------------------------------------------------------------------------------
// Method:      OperatorInterface <<constructor>>
// Description: This will construct and initialize the object.
//              It maps the functions to the buttons/axis.
//---------------------------------------------------------------------------------
TeleopControl::TeleopControl() : m_axisIDs(),
								 m_buttonIDs(),
								 m_controllerIndex(),
								 m_numControllers(0),
								 m_controller()
{
	Initialize();
}

bool TeleopControl::IsInitialized() const
{
	return m_numControllers > 0;
}
void TeleopControl::Initialize()
{
	m_numControllers = 0;
	for ( int inx=0; inx<DriverStation::kJoystickPorts; ++inx )
	{
		m_controller[inx] = nullptr;
		if ( DriverStation::GetJoystickIsXbox( inx ) )
		{
            auto xbox = new DragonXBox( inx );
			m_controller[inx] = xbox;
			m_numControllers++;
		}
		else if ( DriverStation::GetJoystickType( inx ) == GenericHID::kHID1stPerson )
		{
            auto gamepad = new DragonGamepad( inx );
			m_controller[inx] = gamepad;
			m_numControllers++;
		}
	}


    // Initialize the items to not defined
	m_axisIDs.resize(FUNCTION_IDENTIFIER::MAX_FUNCTIONS);
	m_buttonIDs.resize(FUNCTION_IDENTIFIER::MAX_FUNCTIONS);
	m_controllerIndex.resize(FUNCTION_IDENTIFIER::MAX_FUNCTIONS);
    for ( int inx=0; inx<FUNCTION_IDENTIFIER::MAX_FUNCTIONS; ++inx )
    {
        m_axisIDs[inx]    		= IDragonGamePad::UNDEFINED_AXIS;
        m_buttonIDs[inx]  		= IDragonGamePad::UNDEFINED_BUTTON;
        m_controllerIndex[inx]  = -1;
    }

	// @ADDMECH add functions mapping in the next blocks

    auto ctrlNo = 0;
    if ( m_controller[ctrlNo] != nullptr && DriverStation::GetJoystickIsXbox(ctrlNo) )
    {
		m_controllerIndex[ SWERVE_DRIVE_DRIVE]			= ctrlNo;
		m_axisIDs[ SWERVE_DRIVE_DRIVE]					= IDragonGamePad::LEFT_JOYSTICK_Y;
		m_controllerIndex[ SWERVE_DRIVE_STEER]			= ctrlNo;
		m_axisIDs[ SWERVE_DRIVE_STEER]					= IDragonGamePad::LEFT_JOYSTICK_X;
		m_controllerIndex[ SWERVE_DRIVE_ROTATE]			= ctrlNo;
		m_axisIDs[ SWERVE_DRIVE_ROTATE]					= IDragonGamePad::RIGHT_JOYSTICK_X;

		m_controllerIndex[FINDTARGET] 					= ctrlNo;  
		m_buttonIDs[FINDTARGET]	 						= IDragonGamePad::LEFT_BUMPER;	

		m_controllerIndex[ DRIVE_TO_SHOOTING_SPOT ]		= ctrlNo;
		m_buttonIDs[ DRIVE_TO_SHOOTING_SPOT ]			= IDragonGamePad::A_BUTTON;
		m_controllerIndex[ REZERO_PIGEON ]				= ctrlNo;
		m_buttonIDs[ REZERO_PIGEON ]					= IDragonGamePad::B_BUTTON;
		m_controllerIndex[HOLD_POSITION]				= ctrlNo;
		m_buttonIDs[HOLD_POSITION]						= IDragonGamePad::X_BUTTON;

    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("TeleopControl"), string("Initialize"), string("No controller plugged into port 0"));
    }

    ctrlNo = 1;
    if ( m_controller[ctrlNo] != nullptr && DriverStation::GetJoystickIsXbox(ctrlNo) )
    {



	}
    else if ( m_controller[ctrlNo] != nullptr )
    {

	}
	else
	{
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("TeleopControl"), string("Initialize"), string("Controller 1 not handled"));
    }

	ctrlNo = 2;
    if ( m_controller[ctrlNo] != nullptr && DriverStation::GetJoystickIsXbox(ctrlNo) )
    {

	}
    else if ( m_controller[ctrlNo] != nullptr )
    {
	}
	else
	{
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("TeleopControl"), string("Initialize"), string("Controller 2 not handled"));
    }

    ctrlNo = 3;
    if ( m_controller[ctrlNo] != nullptr && DriverStation::GetJoystickIsXbox(ctrlNo) )
    {

	}
    else if ( m_controller[ctrlNo] != nullptr )
    {
	}
	else
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("TeleopControl"), string("Initialize"), string("Controller 3 not handled"));

	}

    ctrlNo = 4;
    if ( m_controller[ctrlNo] != nullptr && DriverStation::GetJoystickIsXbox(ctrlNo) )
    {
	}
    else if ( m_controller[ctrlNo] != nullptr )
    {
	}
	else
	{
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("TeleopControl"), string("Initialize"), string("Controller 4 not handled"));
    }

    ctrlNo = 5;
    if ( m_controller[ctrlNo] != nullptr && DriverStation::GetJoystickIsXbox(ctrlNo) )
    {
	}
    else if ( m_controller[ctrlNo] != nullptr )
    {
	}
	else
	{
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("TeleopControl"), string("Initialize"), string("Controller 5 not handled"));
    }
}


//------------------------------------------------------------------
// Method:      SetAxisScaleFactor
// Description: Allow the range of values to be set smaller than
//              -1.0 to 1.0.  By providing a scale factor between 0.0
//              and 1.0, the range can be made smaller.  If a value
//              outside the range is provided, then the value will
//              be set to the closest bounding value (e.g. 1.5 will
//              become 1.0)
// Returns:     void
//------------------------------------------------------------------
void TeleopControl::SetAxisScaleFactor
(
    TeleopControl::FUNCTION_IDENTIFIER  	function,      // <I> - function that will update an axis
    double                                  scaleFactor    // <I> - scale factor used to limit the range
)
{
	int ctlIndex = m_controllerIndex[ function];
	IDragonGamePad::AXIS_IDENTIFIER axis = m_axisIDs[ function ];
    if ( ctlIndex > -1 && axis != IDragonGamePad::AXIS_IDENTIFIER::UNDEFINED_AXIS  )
    {
    	if (m_controller[ ctlIndex ] != nullptr)
    	{
    		m_controller[ ctlIndex ]->SetAxisScale( axis,scaleFactor);
    	}
    }
}

void TeleopControl::SetDeadBand
(
	TeleopControl::FUNCTION_IDENTIFIER		function,
	IDragonGamePad::AXIS_DEADBAND			deadband    
)
{
	int ctlIndex = m_controllerIndex[ function];
	IDragonGamePad::AXIS_IDENTIFIER axis = m_axisIDs[ function ];
    if ( ctlIndex > -1 && axis != IDragonGamePad::AXIS_IDENTIFIER::UNDEFINED_AXIS  )
    {
    	if (m_controller[ ctlIndex ] != nullptr)
    	{
    		m_controller[ ctlIndex ]->SetAxisDeadband( axis,deadband);
    	}
    }}


//------------------------------------------------------------------
// Method:      SetAxisProfile
// Description: Sets the axis profile for the specifed axis
// Returns:     void
//------------------------------------------------------------------
void TeleopControl::SetAxisProfile
(
    TeleopControl::FUNCTION_IDENTIFIER  function,       // <I> - function that will update an axis
    IDragonGamePad::AXIS_PROFILE        profile         // <I> - profile to use
)
{
	int ctlIndex = m_controllerIndex[ function];
	IDragonGamePad::AXIS_IDENTIFIER axis = m_axisIDs[ function ];
    if ( ctlIndex > -1 && axis != IDragonGamePad::AXIS_IDENTIFIER::UNDEFINED_AXIS  )
    {
    	if (m_controller[ ctlIndex ] != nullptr)
    	{
    		m_controller[ ctlIndex ]->SetAxisProfile( axis,profile);
    	}
    }
}
 
//------------------------------------------------------------------
// Method:      GetAxisValue
// Description: Reads the joystick axis, removes any deadband (small
//              value) and then scales as requested.
// Returns:     double   -  scaled axis value
//------------------------------------------------------------------
double TeleopControl::GetAxisValue
(
    TeleopControl::FUNCTION_IDENTIFIER  function    // <I> - function that whose axis will be read
) const
{
    double value = 0.0;
	int ctlIndex = m_controllerIndex[ function];
	IDragonGamePad::AXIS_IDENTIFIER axis = m_axisIDs[ function ];
    if ( ctlIndex > -1 && axis != IDragonGamePad::AXIS_IDENTIFIER::UNDEFINED_AXIS  )
    {
    	if (m_controller[ ctlIndex ] != nullptr)
    	{
    		value = m_controller[ ctlIndex ]->GetAxisValue( axis );
    	}
    }
    return value;
}

//------------------------------------------------------------------
// Method:      IsButtonPressed
// Description: Reads the button value.  Also allows POV, bumpers,
//              and triggers to be treated as buttons.
// Returns:     bool   -  scaled axis value
//------------------------------------------------------------------
bool TeleopControl::IsButtonPressed
(
    TeleopControl::FUNCTION_IDENTIFIER  function    // <I> - function that whose button will be read
) const
{
    bool isSelected = false;
	int ctlIndex = m_controllerIndex[ function];
	IDragonGamePad::BUTTON_IDENTIFIER btn = m_buttonIDs[ function ];
    if ( ctlIndex > -1 && btn != IDragonGamePad::BUTTON_IDENTIFIER::UNDEFINED_BUTTON  )
    {
    	if (m_controller[ ctlIndex ] != nullptr)
    	{
    		isSelected = m_controller[ ctlIndex ]->IsButtonPressed( btn );
    	}
    }
    return isSelected;
}



void TeleopControl::SetRumble
(
	TeleopControl::FUNCTION_IDENTIFIER  function,       // <I> - controller with this function
	bool                                leftRumble,     // <I> - rumble left
	bool                                rightRumble     // <I> - rumble right
) const
{
	int ctlIndex = m_controllerIndex[ function];
    if ( ctlIndex > -1 )
    {
		SetRumble(ctlIndex, leftRumble, rightRumble);
	}
}

void TeleopControl::SetRumble
(
	int                                 controller,     // <I> - controller to rumble
	bool                                leftRumble,     // <I> - rumble left
	bool                                rightRumble     // <I> - rumble right
) const
{
	if (m_controller[ controller ] != nullptr)
	{
		m_controller[ controller ]->SetRumble(leftRumble, rightRumble);
	}
}

