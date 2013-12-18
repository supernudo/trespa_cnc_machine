/*  
 *  Copyright Droids Corporation (2009)
 *  Olivier MATZ <zer0@droids-corp.org>
 *
 *  Copyright EuRobotics Engineering (2010)
 *  Javier Baliñas Santos <balinas@gmail.com>
 *
 *	Based on revision: sensor.c,v 1.5 2009/05/27 20:04:07 zer0 Exp
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
 *
 */

#include <stdlib.h>

#include <aversive.h>
#include <aversive/error.h>

#include <scheduler.h>
#include <ax12.h>
#include <pid.h>
#include <quadramp.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <blocking_detection_manager.h>

#include <parse.h>
#include <rdline.h>

#include "main.h"
#include "sensor.h"


/* boolean sensors */

struct sensor_filter {
	uint8_t filter;
	uint8_t prev;
	uint8_t thres_off;
	uint8_t thres_on;
	uint8_t cpt;
	uint8_t invert;
};

static struct sensor_filter sensor_filter[SENSOR_MAX] = {
	[S_ALPHA_L_FC] = { 1, 0, 0, 1, 0, 1 }, 	/* 0 */
	[S_ALPHA_R_FC] = { 1, 0, 0, 1, 0, 1 }, 	/* 1 */
	[S_BETA_L_FC] = { 1, 0, 0, 1, 0, 0 }, 	/* 2 */
	[S_BETA_R_FC] = { 1, 0, 0, 1, 0, 0 }, 	/* 3 */
	[S_RESERVED1] = { 0, 0, 0, 1, 0, 0 }, 	/* 4 */
	[S_RESERVED2] = { 0, 0, 0, 1, 0, 0 }, 	/* 5 */
	[S_RESERVED3] = { 0, 0, 0, 1, 0, 0 }, 	/* 6 */
	[S_RESERVED4] = { 0, 0, 0, 1, 0, 0 }, 	/* 7 */

};

/* value of filtered sensors */
static uint16_t sensor_filtered = 0;

/* sensor mapping : 
 * 0:  	S_ALPHA_L_FC RB10
 * 1:  	S_ALPHA_R_FC RB11
 * 2:  	S_BETA_L_FC RA4
 * 3:  	S_BETA_R_FC RC8
 * 4-7: reserved
 */

uint64_t sensor_get_all(void)
{
	uint16_t tmp;
	uint8_t flags;
	IRQ_LOCK(flags);
	tmp = sensor_filtered;
	IRQ_UNLOCK(flags);
	return tmp;
}

uint8_t sensor_get(uint8_t i)
{
	uint64_t tmp = sensor_get_all();
	return (uint8_t)((tmp & ((uint16_t)1 << i))>>i);
}

/* get the physical value of pins */
static uint16_t sensor_read(void)
{
	uint16_t tmp = 0;

	tmp |= (uint16_t)((PORTB & (_BV(11)|_BV(10))) >> 10) << 0;
	tmp |= (uint16_t)((PORTA & (_BV(4))) >> 4) << 2;
	tmp |= (uint16_t)((PORTC & (_BV(8))) >> 8) << 3;
	/* 4 to 7 reserved */	
	/* add reserved sensors here */
	
	return tmp;
}

/* called every 10 ms, see init below */
static void do_boolean_sensors(void *dummy)
{
	uint8_t i;
	uint8_t flags;
	uint16_t sensor = sensor_read();
	uint16_t tmp = 0;

	for (i=0; i<SENSOR_MAX; i++) {
		
		if(sensor_filter[i].filter == 0)
			continue;
		
		if (((uint16_t)1 << i) & sensor) {
			if (sensor_filter[i].cpt < sensor_filter[i].filter)
				sensor_filter[i].cpt++;
			if (sensor_filter[i].cpt >= sensor_filter[i].thres_on)
				sensor_filter[i].prev = 1;
		}
		else {
			if (sensor_filter[i].cpt > 0)
				sensor_filter[i].cpt--;
			if (sensor_filter[i].cpt <= sensor_filter[i].thres_off)
				sensor_filter[i].prev = 0;
		}
		
		if (sensor_filter[i].prev && !sensor_filter[i].invert) {
			tmp |= ((uint16_t)1 << i);
		}
		else if (!sensor_filter[i].prev && sensor_filter[i].invert) {
			tmp |= ((uint16_t)1 << i);
		}

	}
	IRQ_LOCK(flags);
	sensor_filtered = tmp;
	IRQ_UNLOCK(flags);
}



/************ global sensor init */

/* called every 10 ms, see init below */
static void do_sensors(void *dummy)
{
	do_boolean_sensors(NULL);
}

void sensor_init(void)
{
	/* CS EVENT */
	scheduler_add_periodical_event_priority(do_sensors, NULL, 
						10000L / SCHEDULER_UNIT, 
						SENSOR_PRIO);
}

