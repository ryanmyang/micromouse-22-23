/*
 * encoders.c
 */

#include "main.h"
#include "encoders.h"

/*
 * Implement this function so it returns the right encoder value
 */
int16_t getRightEncoderCounts() {
	return ((int16_t) TIM2->CNT);
}

/*
 * Implement this function so it returns the left encoder value
 */
int16_t getLeftEncoderCounts() {
	return ((int16_t) TIM1->CNT);
}

/*
 * This function has already been implemented for you. Enjoy! :)
 */
void resetEncoders() {
	TIM1->CNT = (int16_t) 0;
	TIM2->CNT = (int16_t) 0;
}
