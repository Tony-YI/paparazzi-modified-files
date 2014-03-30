/*
 * for adaptive pid control of a quadcopter.
 * written by Edward, Yi
 * Put this file in paparazzi/sw/airborne/modules/adaptive_pid/
 * with adaptive_pid.h
 * 
 * 
 * 
 */
#include "modules/adaptive_pid/adaptive_pid.h"
#include "state.h"
#include "firmwares/rotorcraft/autopilot.h" 

#ifdef STAB_WINDY_P
PRINT_CONFIG_VAR(STAB_WINDY_P);
#endif

#ifdef STAB_WINDY_I
PRINT_CONFIG_VAR(STAB_WINDY_I);
#endif

#ifdef STAB_WINDY_D
PRINT_CONFIG_VAR(STAB_WINDY_D);
#endif

#ifdef STAB_WINDY_DD
PRINT_CONFIG_VAR(STAB_WINDY_DD);
#endif

bool_t adaptive_pid_Windy;
#ifdef USE_ADAPTIVE_PID_MANUAL
bool_t Is_Manual = 1;
bool_t prev_Comarasion = 0;
#else
bool_t Is_Manual = 0;
bool_t prev_Comarasion = 0;
#endif

void adaptive_pid_init(void)
{
	adaptive_pid_Windy = 0;
	return ;
}

void adaptive_pid_periodic(void)
{
	/*
	 * 
	 * If the quadrotor is in windy envirnoment, change the PID value of the attitude loop.
	 * 
	 * 
	 * 
	 * 
	 * 
	 * 
	 */
	 
	 adaptive_pid_Is_Windy();
	 //If the quadcopter is in windy envirnoment
	 if(adaptive_pid_Windy)//Preoccupy the condition statement
	 {
		 ADAPTIVE_PID_WINDY_CONDITION();
	 }
	 else
	 {
		 ADAPTIVE_PID_NORMAL_CONDITION();
	 }
	return ;
}

void adaptive_pid_Is_Windy(void)
{
	#ifdef USE_ADAPTIVE_PID_MANUAL
	{	//CHECK FOR THE 7th CHANNEL of the radio control for tuning
		PRINT_CONFIG_MSG("USING MANUAL TUNING");
		if(autopilot_mode == AP_MODE_ADAPTIVE_PID)
		{
			if(prev_Comarasion && (radio_control.values[RADIO_MANUAL_SWITCH] < MANUAL_SWITCH_THRESHOLD))
			{
				adaptive_pid_Windy = !adaptive_pid_Windy;
			}
			prev_Comarasion = radio_control.values[RADIO_MANUAL_SWITCH] < MANUAL_SWITCH_THRESHOLD;
			DOWNLINK_SEND_ADAPTIVE_PID_STATUS(DefaultChannel, DefaultDevice, &Is_Manual, &adaptive_pid_Windy);
		}
		else
		adaptive_pid_Windy = 0;
    }
	#else
	{
		if(autopilot_mode == AP_MODE_ADAPTIVE_PID)
		{
			DOWNLINK_SEND_ADAPTIVE_PID_STATUS(DefaultChannel, DefaultDevice, &Is_Manual, &adaptive_pid_Windy);
			state.body_rates_i;
		}
		else
		;
	}
	#endif
	
	
}
