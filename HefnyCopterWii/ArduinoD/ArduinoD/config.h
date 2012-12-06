/*
 * config.h
 *
 * Created: 04-Dec-12 9:16:00 AM
 *  Author: M.Hefny
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_




#define QUADCOPTER
//#define HEXACOPTER

#ifdef QUADCOPTER
#define NUMBER_OF_MOTORS	4
#endif
#ifdef HEXACOPTER
#define NUMBER_OF_MOTORS	6
#endif

#ifdef OCTACOPTER
#define NUMBER_OF_MOTORS	8
#endif

#endif /* CONFIG_H_ */


