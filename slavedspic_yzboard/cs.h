/*  
 *  Copyright Droids Corporation
 *  Olivier Matz <zer0@droids-corp.org>
 * 
 *  Copyright EuRobotics Engineering (2010)
 *  Javier Bali�as Santos <balinas@gmail.com>
 *
 *  Based on revision : cs.h,v 1.3 2009/03/29 18:42:41 zer0 Exp
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

void dspic_cs_init(void);
void do_cs(void *dummy);
void dump_cs(const char *name, struct cs *cs);
void dump_cs_debug(const char *name, struct cs *cs);
void dump_pid(const char *name, struct pid_filter *pid);
