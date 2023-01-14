/*
 * irs.h
 */

#ifndef INC_IRS_H_
#define INC_IRS_H_

// The number of samples to take
#define NUM_SAMPLES 128

// Using this enumeration makes the code more readable
typedef enum
{
	IR_LEFT = 0,
	IR_FRONT_LEFT = 1,
	IR_FRONT_RIGHT = 2,
	IR_RIGHT = 3
}IR;

uint16_t readIR(IR ir);
uint16_t readLeftIR(void);
uint16_t readFrontLeftIR(void);
uint16_t readFrontRightIR(void);
uint16_t readRightIR(void);
uint16_t analogRead(IR ir);

#endif /* INC_IRS_H_ */