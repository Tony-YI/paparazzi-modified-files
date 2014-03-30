/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/datalink.c
 * Handling of messages coming from ground and other A/Cs.
 *
 */

#define DATALINK_C
#define MODULES_DATALINK_C

#include "subsystems/datalink/datalink.h"

#include "generated/modules.h"

#include "generated/settings.h"
#include "subsystems/datalink/downlink.h"
#include "messages.h"
#include "dl_protocol.h"
#include "mcu_periph/uart.h"

#ifdef BOOZ_FMS_TYPE
#include "booz_fms.h"
#endif

#if defined RADIO_CONTROL && defined RADIO_CONTROL_TYPE_DATALINK
#include "subsystems/radio_control.h"
#endif

#include "firmwares/rotorcraft/navigation.h"

#include "math/pprz_geodetic_int.h"
#include "subsystems/ins.h"

#define IdOfMsg(x) (x[1])

void dl_parse_msg(void) {

  datalink_time = 0;

  uint8_t msg_id = IdOfMsg(dl_buffer);
  switch (msg_id) {

  case  DL_PING:
    {
      DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
    }
    break;

  case DL_SETTING :
    {
      if (DL_SETTING_ac_id(dl_buffer) != AC_ID) break;
      uint8_t i = DL_SETTING_index(dl_buffer);
      float var = DL_SETTING_value(dl_buffer);
      DlSetting(i, var);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &var);
    }
    break;

  case DL_GET_SETTING :
    {
      if (DL_GET_SETTING_ac_id(dl_buffer) != AC_ID) break;
      uint8_t i = DL_GET_SETTING_index(dl_buffer);
      float val = settings_get_value(i);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
    }
    break;

/* this case for DL_HOVER_POINT_CHANGE_ECEF is added by Yi and Edward */
/*********************************************************************/
  case DL_HOVER_POINT_CHANGE_ECEF:
    {  
      uint8_t ac_id = DL_HOVER_POINT_CHANGE_ECEF_ac_id(dl_buffer);
      if(ac_id != AC_ID) break;
      uint8_t external_gps_fixed = DL_HOVER_POINT_CHANGE_ECEF_external_gps_fix(dl_buffer);

      /** getting the ecef coordinate from external gps **/
      struct EcefCoor_i external_gps_ecef_pos;
      external_gps_ecef_pos.x = DL_HOVER_POINT_CHANGE_ECEF_ecef_pos_x(dl_buffer);//DL_HOVER_POINT_CHANGE_ECEF_ecef_pos_x() is defined in dl_protocol.h
      external_gps_ecef_pos.y = DL_HOVER_POINT_CHANGE_ECEF_ecef_pos_y(dl_buffer);
      external_gps_ecef_pos.z = DL_HOVER_POINT_CHANGE_ECEF_ecef_pos_z(dl_buffer);

      DOWNLINK_SEND_HOVER_POINT_CHANGE_ECEF(DefaultChannel, DefaultDevice, &ac_id, &external_gps_fixed, &(external_gps_ecef_pos.x), &(external_gps_ecef_pos.y), &(external_gps_ecef_pos.z));

      if(external_gps_fixed == 0x03)//case for 3D_Fix.
      {
        if(!ins_ltp_initialised || guidance_h_mode != GUIDANCE_H_MODE_HOVER) break;//ins_ltp_initialised defined in ins_int.h 
                                                                                   //no HMOE reference
                                                                                   //not in the HOVER_Z_HOLD mode
		                                                                   //not in flight(looks like no needed)	  
	struct NedCoor_i external_gps_ned_pos;
	memset((void *) &external_gps_ned_pos,0,sizeof(struct NedCoor_i));//just in case, clear the memory.
		  
	/** convert the ecef coordinate into ned(ltp) coordinate **/
	ned_of_ecef_point_i(&external_gps_ned_pos,&ins_ltp_def,&external_gps_ecef_pos);
		  
		  
	/** change the current setpoint with external_gps_ned_pos coordinate **/
	struct Int32Vect2 external_gps_h_pos;
	//cm to m and store in external_gps_h_pos
	//check function ins_update_gps() for reference.
        INT32_VECT2_SCALE_2(external_gps_h_pos,external_gps_ned_pos,INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
	//subsitute the guidance_h_pos_sp
	VECT2_COPY(guidance_h_pos_sp,external_gps_h_pos);
	/*
	 * the above code is used for congruity, but waste space and time.
	 * simple using
	 * 
	 * guidance_h_pos_sp.x = external_gps_ned_pos.x;
	 * guidance_h_pos_sp.y = external_gps_ned_pox.y;
	 * 
	 * can achieve the same effect.
	*/
      }
      
      else break;//hover pos_sp no change
    } 
/**------------------------------------------------------------------*/


#if defined USE_NAVIGATION
  case DL_BLOCK :
    {
      if (DL_BLOCK_ac_id(dl_buffer) != AC_ID) break;
      nav_goto_block(DL_BLOCK_block_id(dl_buffer));
    }
    break;

  case DL_MOVE_WP :
    {
      uint8_t ac_id = DL_MOVE_WP_ac_id(dl_buffer);
      if (ac_id != AC_ID) break;
      uint8_t wp_id = DL_MOVE_WP_wp_id(dl_buffer);
      struct LlaCoor_i lla;
      struct EnuCoor_i enu;
      lla.lat = INT32_RAD_OF_DEG(DL_MOVE_WP_lat(dl_buffer));
      lla.lon = INT32_RAD_OF_DEG(DL_MOVE_WP_lon(dl_buffer));
      /* WP_alt is in cm, lla.alt in mm */
      lla.alt = DL_MOVE_WP_alt(dl_buffer)*10 - ins_ltp_def.hmsl + ins_ltp_def.lla.alt;
      enu_of_lla_point_i(&enu,&ins_ltp_def,&lla);
      enu.x = POS_BFP_OF_REAL(enu.x)/100;
      enu.y = POS_BFP_OF_REAL(enu.y)/100;
      enu.z = POS_BFP_OF_REAL(enu.z)/100;
      VECT3_ASSIGN(waypoints[wp_id], enu.x, enu.y, enu.z);
      DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id, &enu.x, &enu.y, &enu.z);
    }
    break;
#endif /* USE_NAVIGATION */
#ifdef RADIO_CONTROL_TYPE_DATALINK
  case DL_RC_3CH :
#ifdef RADIO_CONTROL_DATALINK_LED
    LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
#endif
    parse_rc_3ch_datalink(
        DL_RC_3CH_throttle_mode(dl_buffer),
        DL_RC_3CH_roll(dl_buffer),
        DL_RC_3CH_pitch(dl_buffer));
    break;
  case DL_RC_4CH :
#ifdef RADIO_CONTROL_DATALINK_LED
    LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
#endif
    parse_rc_4ch_datalink(
        DL_RC_4CH_mode(dl_buffer),
        DL_RC_4CH_throttle(dl_buffer),
        DL_RC_4CH_roll(dl_buffer),
        DL_RC_4CH_pitch(dl_buffer),
        DL_RC_4CH_yaw(dl_buffer));
    break;
#endif // RADIO_CONTROL_TYPE_DATALINK
  default:
    break;
  }
  /* Parse modules datalink */
  modules_parse_datalink(msg_id);
}
