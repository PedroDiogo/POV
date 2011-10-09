#ifndef POV_H
#define POV_H

// #define POV_TIMER 154	// 450Hz
#define POV_TIMER 141	// 400Hz
// #define POV_TIMER 124	// 350Hz
// #define POV_TIMER 102	// 300Hz
// #define POV_TIMER 24		// 200Hz

void testLEDs();
void setLEDs(unsigned char pattern);
void POVRoutine(void);

#endif