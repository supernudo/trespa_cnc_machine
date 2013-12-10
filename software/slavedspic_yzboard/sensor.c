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
	[S_Y_CALIB] 	= { 1, 0, 0, 1, 0, 0 }, 	/* 0 */
	[S_Y_FC_R] 		= { 1, 0, 0, 1, 0, 0 }, 	/* 1 */
	[S_Y_FC_L] 		= { 1, 0, 0, 1, 0, 0 }, 	/* 2 */
	[S_Z_CALIB] 	= { 1, 0, 0, 1, 0, 0 }, 	/* 3 */
	[S_Z_FC_UP] 	= { 1, 0, 0, 1, 0, 0 }, 	/* 4 */
	[S_Z_FC_DOWN] 	= { 1, 0, 0, 1, 0, 0 }, 	/* 5 */
	[S_RESERVED1] 	= { 1, 0, 0, 1, 0, 0 }, 	/* 6 */
	[S_RESERVED2] 	= { 1, 0, 0, 1, 0, 0 }, 	/* 7 */
};


/* value of filtered sensors */
static uint16_t sensor_filtered = 0;

/* sensor mapping : 
 * 0:  	S_Y_CALIB 	RC1
 * 1:  	S_Y_FC_L 	RB12
 * 2:  	S_Y_FC_R 	RB13
 * 3:  	S_Z_CALIB 	RB11
 * 4:  	S_Z_FC_UP 	RB10
 * 5:  	S_Z_FC_DOWN	RB2
 * 6: 	reserved		RA8
 * 7:		reserved		RC3
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
	tmp |= (uint16_t)((PORTC & (_BV(1)))  >> 1)  << 0;
	tmp |= (uint16_t)((PORTB & (_BV(12))) >> 12) << 2;
	tmp |= (uint16_t)((PORTB & (_BV(13))) >> 13) << 1;
	tmp |= (uint16_t)((PORTB & (_BV(11))) >> 11) << 3;
	tmp |= (uint16_t)((PORTB & (_BV(10))) >> 10) << 5;
	tmp |= (uint16_t)((PORTB & (_BV(2)))  >> 2)  << 4;

	/* 6 to 7 reserved */	
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

/* called periodically */
void do_sensors(void *dummy)
{
	do_boolean_sensors(NULL);
}

void sensor_axis_z_enable_calib(void)
{
	uint8_t flags;

	IRQ_LOCK(flags);
	IFS0bits.IC1IF = 0; 	// clear IC1 Interrupt Status Flag	IEC0bits.IC1IE = 1; 	// enable IC1 interrupt
	IRQ_UNLOCK(flags);
}

void sensor_axis_z_disable_calib(void)
{
	uint8_t flags;

	IRQ_LOCK(flags);
	IFS0bits.IC1IF = 0; 	// clear IC1 Interrupt Status Flag	IEC0bits.IC1IE = 0; 	// disable IC1 interrupt
	IRQ_UNLOCK(flags);
}

void sensor_axis_y_enable_calib(void)
{
	uint8_t flags;

	IRQ_LOCK(flags);
	IFS0bits.IC2IF = 0; 	// clear IC1 Interrupt Status Flag	IEC0bits.IC2IE = 1; 	// enable IC1 interrupt
	IRQ_UNLOCK(flags);
}

void sensor_axis_y_disable_calib(void)
{
	uint8_t flags;

	IRQ_LOCK(flags);
	IFS0bits.IC2IF = 0; 	// clear IC1 Interrupt Status Flag	IEC0bits.IC2IE = 0; 	// disable IC1 interrupt
	IRQ_UNLOCK(flags);
}

void sensor_init(void)
{
	/* config inputs interrups for yz axes calibration */

	/* initialize input capture (IC) 1 and 2 for calib sensors events */

	/* 
		01  - falling and rising edge
		10  - falling edge
		11  - rising edge		
	*/
	IC1CONbits.ICM =0b000;		// disable Input Capture module	IC1CONbits.ICTMR = 1; 		// select Timer2 as the IC time base	IC1CONbits.ICI = 0b00; 		// interrupt on every capture event	IC1CONbits.ICM = 0b011; 	// generate capture event on edge event 
		
	IC2CONbits.ICM =0b00; 		// disable Input Capture module	IC2CONbits.ICTMR = 1; 		// select Timer2 as the IC time base	IC2CONbits.ICI = 0b00; 		// interrupt on every capture event	IC2CONbits.ICM = 0b011; 	// generate capture event on edge event
	/* setup interrupts priority level higher than scheduler! */	IPC0bits.IC1IP = 5;
	IPC1bits.IC2IP = 5;

	sensor_axis_z_disable_calib();
	//sensor_axis_z_enable_calib();
	sensor_axis_y_disable_calib();
	//sensor_axis_y_enable_calib();
}


/* input interrupt connected to CALIB_Z */void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void){	uint8_t flags;	_IC1IF=0;	IRQ_LOCK(flags);	encoders_dspic_set_value(ENCODER_Z, 0);	IRQ_UNLOCK(flags);}

/* input compare 2 interrupt connected to CALIB_Y */void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void){	uint8_t flags;	_IC2IF=0;	IRQ_LOCK(flags);	encoders_dspic_set_value(ENCODER_Y, 0);	IRQ_UNLOCK(flags);}
