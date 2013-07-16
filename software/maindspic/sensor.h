/*  
 *  Copyright Droids Corporation (2009)
 *  Olivier MATZ <zer0@droids-corp.org>
 *
 *  Copyright EuRobotics Engineering (2010)
 *  Javier Baliñas Santos <balinas@gmail.com>
 *
 *	Based on revision: sensor.h,v 1.5 2009/05/27 20:04:07 zer0 Exp
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


/* synchronize with sensor.c */
#define S_X_MOT_L_FC 	0
#define S_X_MOT_R_FC	1
#define S_RESERVED1    	2
#define S_RESERVED2    	3
#define S_RESERVED3    	4
#define S_RESERVED4    	5
#define S_RESERVED5    	6
#define S_RESERVED6    	7

#define SENSOR_MAX	8

void sensor_init(void);

/* get filtered values of boolean sensors */
uint64_t sensor_get_all(void);
uint8_t sensor_get(uint8_t i);

