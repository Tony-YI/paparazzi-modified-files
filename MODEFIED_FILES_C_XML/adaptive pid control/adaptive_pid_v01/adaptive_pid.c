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

 
#ifdef STAB_WINDY_P
PRINT_CONFIG_VAR(STAB_WINDY_P);
#endif

#ifdef STAB_WINDY_I
PRINT_CONFIG_VAR(STAB_WINDY_I);
#endif

#ifdef STAB_WINDY_D
PRINT_CONFIG_VAR(STAB_WINDY_D);
#endif

bool_t adaptive_pid_Windy;

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
	#ifdef ADAPTIVE_PID_MANUAL
		//CHECK FOR THE 7th CHANNEL of the radio control for tuning 
	#else
	
	#endif
}
