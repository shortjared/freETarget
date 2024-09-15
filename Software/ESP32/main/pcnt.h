/*----------------------------------------------------------------
 *
 * pcnt.h
 *
 * Header file for Pulse Counter Module
 *
 *---------------------------------------------------------------*/
#ifndef _PCNT_H_
#define _PCNT_H_

/*
 * Global functions
 */
void pcnt_init(int unit, int control, int signal); // pcnt Control
int pcnt_read(unsigned int unit);                  // Read timer contents
void pcnt_clear(void);                             // Clear the timer contents
void pcnt_test(int which_test);                    // Trigger the counters and verify operation
void pcnt_cal(void);                               // Trigger the counters print the time delay

/*
 * Typedefs
 */

/*
 * Definitions
 */
#define NORTH_LO 0 // Time location in timer array
#define EAST_LO 1
#define SOUTH_LO 2
#define WEST_LO 3
#define NORTH_HI 4
#define EAST_HI 5
#define SOUTH_HI 6
#define WEST_HI 7

#define PCNT_NOT_TRIGGERED 200 // Ignore any value over 200 counts

#endif
