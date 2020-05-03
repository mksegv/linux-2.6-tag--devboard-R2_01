/*
 * Copyright (c) 2004 Axis Communications AB
 *
 * Authors: Peter Kjellerstedt
 *
 */

#ifndef SYS_TIME_H
#define SYS_TIME_H

/****************** INCLUDE FILES SECTION ***********************************/

/****************** CONSTANT AND MACRO SECTION ******************************/

/* The following four macros were borrowed from sys/time.h as they make
 * working with struct timeval so much easier.
 */

#define timerisset(tvp)	((tvp)->tv_sec || (tvp)->tv_usec)

#define timercmp(a, b, CMP)                                                  \
	(((a)->tv_sec == (b)->tv_sec) ?                                      \
	 ((a)->tv_usec CMP (b)->tv_usec) :                                   \
	 ((a)->tv_sec CMP (b)->tv_sec))

#define timeradd(a, b, result)                                               \
	do {                                                                 \
		(result)->tv_sec = (a)->tv_sec + (b)->tv_sec;                \
		(result)->tv_usec = (a)->tv_usec + (b)->tv_usec;             \
		if ((result)->tv_usec >= 1000000) {                          \
			++(result)->tv_sec;                                  \
			(result)->tv_usec -= 1000000;                        \
		}                                                            \
	} while (0)

#define timersub(a, b, result)                                               \
	do {                                                                 \
		(result)->tv_sec = (a)->tv_sec - (b)->tv_sec;                \
		(result)->tv_usec = (a)->tv_usec - (b)->tv_usec;             \
		if ((result)->tv_usec < 0) {                                 \
			--(result)->tv_sec;                                  \
			(result)->tv_usec += 1000000;                        \
		}                                                            \
	} while (0)

/****************** TYPE DEFINITION SECTION *********************************/

/****************** EXPORTED FUNCTION DECLARATION SECTION *******************/

#endif
/****************** END OF FILE sys_time.h **********************************/
