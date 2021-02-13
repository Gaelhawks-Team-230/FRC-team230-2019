/*
 * Common.h
 *
 *  Created on: Jan 13, 2019
 *      Author: Gaelhawks
 */
//19 11/16ths

#ifndef SRC_COMMON_H_
#define SRC_COMMON_H_

#define USE_GYRO
//#define CHECK_LOOPTIME
//#define USE_PIXY_TARGET
#define USE_PIXY      // note this is for line pixy only
#define USE_USB_CAMERA
//#define PRACTICE_BOT
#define STRAFE_TRACKING		// comment this out to use Rotate Tracking
#define USE_VACUUM


#include "TalonXX_main.h"


/*******************\
 *  Global Values  *
\*******************/
#define PI	 						(3.14159265)
#define RAD_TO_DEG 					(180.0/PI)
#define DEG_TO_RAD					(PI/180.0)
#define X_SCALING_FACTOR 			(1)
#define Y_SCALING_FACTOR			(0.65)

#define JOYSTICK_DEADBAND			(0.3)
#define DRIVE_DEADBAND				(0.07)//(0.1)
#define GAMEPAD_DEADBAND			(0.5)

#define LOOPTIME 					(0.02)
#define PIXYTIME					(0.008)
#define SAMPLES_PER_SECOND			(1.0/LOOPTIME)
#define N1SEC  						((int) SAMPLES_PER_SECOND)

#ifdef USE_PIXY
#define TARGET_PIXY_I2C_ADDRESS     (int)(0x54)
#endif

/*
typedef enum
{
	// example: USB_JOY_DRIVE	= 1,
	// example: USB_GAMEPAD		= 2,
	
} usbs;
*/

typedef enum
{
	CLAW_PDP 		= 2,

} pdp;

typedef enum
{
	PWM_CLAW_WHEELS			= 0,
	PWM_CLAW_TILT			= 1,
	PWM_HATCH_CAMS			= 2,
	PWM_CLIMBER				= 3,
	PWM_ELEVATOR_ONE		= 4,
	PWM_VACUUM        		= 5,
} pwms;

typedef enum
{
    CAN_FRONT_LEFT        = 1,
    CAN_FRONT_RIGHT       = 3,
    CAN_BACK_LEFT         = 2,
    CAN_BACK_RIGHT        = 0,
} can;

typedef enum
{
	//example GYRO_ANALOG_INPUT   = 1,
	//example GRAB_ANALOG_INPUT   = 0,
	PIXY_LINE_ANALOG_INPUT  	=  0,

} analogs;

/*
typedef enum
{
//	RELAY_CLAW_DROP			                = 0,

}relays;
*/

typedef enum
{
	FRONT_LEFT_ENCODER_ONE						= 0,
	FRONT_LEFT_ENCODER_TWO						= 1,
	FRONT_RIGHT_ENCODER_ONE						= 2,
	FRONT_RIGHT_ENCODER_TWO						= 3,
	BACK_LEFT_ENCODER_ONE						= 4,
	BACK_LEFT_ENCODER_TWO						= 5,
	BACK_RIGHT_ENCODER_ONE						= 6, //6
	BACK_RIGHT_ENCODER_TWO						= 7, //7
	
	ELEVATOR_ENCODER_ONE						= 8,
	ELEVATOR_ENCODER_TWO						= 9,
	DIGIN_CARGO_SENSOR							= 10,  //MXP PWM 0 
	CLIMBER_ENCODER_ONE							= 14,  //MXP DIO 4,5
	CLIMBER_ENCODER_TWO							= 15,
	CLAW_ANGLE_ENCODER_CHANNEL_A				= 16, //16 //MXP DIO 6,7
	CLAW_ANGLE_ENCODER_CHANNEL_B				= 17, //17
	PIXY_LINE_DETECTOR							= 11,  //MXP PWM 1
	HATCH_CAM_SENSOR							= 12,  //MXP PWM 2

	LED_LIGHT_OUTPUT_1 							= 19,  //MXP PWM 5 //21  
	LED_LIGHT_OUTPUT_2							= 20,  //MXP PWM 6 //22
	LED_LIGHT_OUTPUT_3							= 21,  //MXP PWM 7 //23

} digitals;

/*
** List of gamepad (USB_GAMEPAD below) button and axis assignments
*/

typedef enum
{
	ENABLE_GYRO_BUTTON 		= 1,
	CLIMB_TWO_PREP		    = 2, // was 4
	OVERRIDE_BUTTON_RIGHT   = 4, // was 2
	AUTO_PAUSE_BUTTON       = 16,
	TRACKING_BUTTON			= 3,
	CLIMB_CHECK_BUTTON		= 5,
	CLIMB_LEV3_BUTTON		= 14,
	OVERRIDE_MAIN			= 8,
	OVERRIDE_CLIMBER		= 6,
	OVERRIDE_ELEVATOR		= 10,
	OVERRIDE_PIVOT			= 12,
	DRAG_BUTTON 			= 15,
} joystick_buttons;

typedef enum
{
	SPEED_AXIS					= 1,
	STRAFE_AXIS					= 0,
	ROTATE_AXIS					= 5,
	TRACKING_AXIS				= 2,
	TILT_STOP_LIMIT				= 6,
} joystick_axes;

typedef enum
{
	//GAMEPAD_SPEED_AXIS          = 3, //1,
	ELEVATOR_AXIS				= 3,
	TILT_AXIS					= 1,
} gamepad_axes;

typedef enum
{
	DPAD_CLIMBER_MANUAL_UP				= 0,
	DPAD_CLIMBER_MANUAL_DOWN			= 180,
	DPAD_CAM_RELEASE					= 270,
	//DPAD_CLIMB_RETRACT						= 90,
	DPAD_STOP_MECHANISMS				= 90,
} gamepad_pov;

typedef enum
{
//	PIXY_TEST_BUTTON 			= 1,  //Button X?
	HATCH_PICKUP_BUTTON		    = 2,
	ELEVAOTR_LOW_BUTTON			= 1,
	ELEVATOR_MIDDLE_BUTTON		= 3,
	ELEVATOR_TOP_BUTTON			= 4,
	FEEDER_STATION_BUTTON 		= 10,
	WHEELS_IN_BUTTON			= 8,
	WHEELS_OUT_BUTTON			= 7,
	PIVOT_UP_BUTTON				= 6,
	PIVOT_DOWN_BUTTON			= 5,
	CARGO_MOD					= 9,
} gamepad_buttons;




#endif /* SRC_COMMON_H_ */
