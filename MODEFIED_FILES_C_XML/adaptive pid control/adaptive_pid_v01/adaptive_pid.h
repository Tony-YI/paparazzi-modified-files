/*
 * for adaptive pid control of a quadcopter.
 * written by Edward, Yi
 * Put this file in paparazzi/sw/airborne/modules/adaptive_pid/
 * 
 * 
 * 
 * 
 */

#ifndef _ADAPTIVE_PID_H
#define _ADAPTIVE_PID_H


#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance.h"
#include "generated/airframe.h"
#include "math/pprz_algebra_int.h"

#ifdef STABILIZATION_ATTITUDE_QUAT_INT_H //ONLY WORKS UNDER STABLIZATION_ATTITUDE_QUAT_INT NOW.


extern bool_t adaptive_pid_Windy;
extern void adaptive_pid_init(void);//inilization of the adaptive pid mode.
extern void adaptive_pid_periodic(void);//periodic function of the adaptive pid
extern void adaptive_pid_Is_Windy(void);//Check for windy condition, set to 1 if is windy.


//CHANGE THE attitude loop into wind-tolarable mode
#define ADAPTIVE_PID_WINDY_CONDITION() {    \
	stabilization_gains.p.x = STAB_WINDY_P; \
	stabilization_gains.p.y = STAB_WINDY_P; \
	stabilization_gains.i.x = STAB_WINDY_I; \
	stabilization_gains.i.y = STAB_WINDY_I; \
	stabilization_gains.d.x = STAB_WINDY_D; \
	stabilization_gains.d.y = STAB_WINDY_D; \
}

//CHANGE THE attitude loop into normal mode
#define ADAPTIVE_PID_NORMAL_CONDITION(){    \
    stabilization_gains.p.x = STABILIZATION_ATTITUDE_PHI_PGAIN; \
	stabilization_gains.p.y = STABILIZATION_ATTITUDE_THETA_PGAIN; \
	stabilization_gains.i.x = STABILIZATION_ATTITUDE_PHI_IGAIN; \
	stabilization_gains.i.y = STABILIZATION_ATTITUDE_THETA_IGAIN; \
	stabilization_gains.d.x = STABILIZATION_ATTITUDE_PHI_DGAIN; \
	stabilization_gains.d.y = STABILIZATION_ATTITUDE_THETA_DGAIN; \
}


#endif //ONLY WORKS UNDER STABLIZATION_ATTITUDE_QUAT_INT NOW.
#endif //HEADER FILE PROTECTOR.
