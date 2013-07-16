/*  
 *  Copyright Droids Corporation (2009)
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id: actuator.c,v 1.3 2009/05/02 10:08:09 zer0 Exp $
 *
 */

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <encoders_dspic.h>
#include <ax12.h>
#include <scheduler.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>

#include "main.h"
#include "actuator.h"
#include "ax12_user.h"


void ax12_set_and_save(void *ax12_id, int32_t val)
{
	/* we need to do the saturation here, before saving the
	 * value */
	if (val > 1023)
		val = 1023;
	if (val < -1023)
		val = -1023;

	/* set polarity and save value*/
	if((uint16_t)ax12_id == ALPHA_AX12){
		val = -val;
		slavedspic.ax12_alpha_speed = val;
	}
	else if((uint16_t)ax12_id == BETA_AX12){
		val = -val;
		slavedspic.ax12_beta_speed = val;
	}	
	
	/* set ax12 sign and apply*/
	if(val < 0){
		val = -val;
		val |= 0x0400;
	}

	ax12_user_write_int(&gen.ax12, (uint16_t)ax12_id,
					    AA_MOVING_SPEED_L, (uint16_t)val);

}
