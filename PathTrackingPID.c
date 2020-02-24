/**********************************************************************
* Course: Embedded Systems 3 (ES3)
* Assignment: Driving along the wall
* Week: 6
* 
* Authors: 
*	Yasser Alhazmi (3529495)
*	Sam Jarvis (3734811)
*
* Description: Implements driving along the wall using PID controller 
*			   system and implementing assignments of previous weeks
**********************************************************************/

/*****************************************************************************/
// Includes:
#include "RP6ControlLib.h"
#include "RP6I2CmasterTWI.h"
#include <math.h>
/*****************************************************************************/
//Global variabbles
int cursorPosition = 0; //used for maf
uint16_t total = 0; //used for maf

/* Struct to allow changing values using buttons */
struct changingValues {
int speed;
int Kp;
int Kd;
int window;
} screenvalues;

/*****************************************************************************/

/* I2C Error handler
 * This function gets called automatically if there was an I2C Error like
 * the slave sent a "not acknowledge" (NACK, error codes e.g. 0x20 or 0x30).
 * The most common mistakes are:
 *   - using the wrong address for the slave
 *   - slave not active or not connected to the I2C-Bus
 *   - too fast requests for a slower slave
 * Be sure to check this if you get I2C errors!
 */
void I2C_transmissionError(uint8_t errorState){
	writeString_P("\nI2C ERROR - TWI STATE: 0x");
	writeInteger(errorState, HEX);
	//writeChar('\n');
	mSleep(500);
	sound(180,80,25);
	sound(220,80,25);
}

/* The moving average filter, detailed comments are removed from here because they were supplied in W5 assignment */
uint16_t MovingAverage(uint16_t readingValue, uint16_t *arrayOfReadings, int window){
  total = total - arrayOfReadings[cursorPosition];
  arrayOfReadings[cursorPosition] = readingValue;
  total = total + arrayOfReadings[cursorPosition];
  cursorPosition = (cursorPosition + 1) % window;
  return total / window;
}

/* The transfer function */
int16_t transferFunction(int sharpReading){
	uint16_t voltage = (sharpReading / 1024) * 5; // converts sharp reading to voltage because the transfer function converts voltage to distance.
	uint16_t distance = 8.7696 * (pow(voltage, -0.973)); // implements the transfer fucntion with the voltage
	return distance; // returns the result of the transfer function
}

/* Increasing the values of one of the screen variables using buttons */
void increaseValue(int tmp){
switch (tmp){
	case 0:
		screenvalues.speed += 5;
		break;
	case 1:
		screenvalues.Kp += 1;
		break;
	case 2:
		screenvalues.Kd += 1;
		break;
	case 3:
		screenvalues.window += 5;
		break;
	default:
		break;}
	}

/* Decreasing the values of one of the screen variables using buttons */
void decreaseValue(int tmp){
switch (tmp){
	case 0:
		screenvalues.speed -= 5;
		break;
	case 1:
		screenvalues.Kp -= 1;
		break;
	case 2:
		screenvalues.Kd -= 1;
		break;
	case 3:
		screenvalues.window -= 5;
		break;
	default:
		break;}
}

/* Printing the values when the 'next' button is pressed */
void printValue(int tmp){
			setCursorPosLCD(0, 10);
			writeStringLCD("        ");
			setCursorPosLCD(1, 10);
			writeStringLCD("        ");
switch (tmp){
	case 0:
			setCursorPosLCD(0, 10);
			writeStringLCD("Speed");
			setCursorPosLCD(1, 10);
			writeIntegerLCD(screenvalues.speed, DEC);
			break;
	case 1:
			setCursorPosLCD(0, 10);
			writeStringLCD("Kp");
			setCursorPosLCD(1, 10);
			writeIntegerLCD(screenvalues.Kp, DEC);
			break;
	case 2:
			setCursorPosLCD(0, 10);
			writeStringLCD("Kd");
			setCursorPosLCD(1, 10);
			writeIntegerLCD(screenvalues.Kd, DEC);
			break;
	case 3:
			setCursorPosLCD(0, 10);
			writeStringLCD("window");
			setCursorPosLCD(1, 10);
			writeIntegerLCD(screenvalues.window, DEC);
			break;
	default:
		break;}
}

int main(void){
	initRP6Control(); // Always call this first! The Processor will not work correctly otherwise.
	initLCD(); // Initialize the LC-Display (LCD)
	I2CTWI_initMaster(100); // Initialize the TWI Module for Master operation

	// Register the event handlers:
	I2CTWI_setTransmissionErrorHandler(I2C_transmissionError);

	/* Declaring essential variables */
	int error =0, error2=0, sumError=0, sumError2=0, speedCorrection2=0, lastError2=0, speedCorrection=0, lastError=0, tmp=0;
	uint16_t desiredDistance = 155;
	uint16_t queue[MAF_MAXSIZE] = {0};

	screenvalues.speed = 20;
	screenvalues.Kp = 1;
	screenvalues.Kd = 1;
	screenvalues.window = 15;

while(true){
	int key = checkReleasedKeyEvent();
	uint16_t ReadAdc2 = readADC(ADC_2); //Reading from ADC
	uint16_t ReadAdc3 = readADC(ADC_3); //Reading from ADC

    /* Uses distances caluclated by the transfer fucntion for the moving average filter. */
	uint16_t distance2 = transferFunction(ReadAdc2);
	uint16_t distance3 = transferFunction(ReadAdc3);

	uint16_t averageAdc2 = MovingAverage(distance2, queue, screenvalues.window);
	uint16_t averageAdc3 = MovingAverage(distance3, queue, screenvalues.window);

	if(key == 5)
		{
			if(tmp < 4){tmp++;} 
			else {tmp = 0;}
		}

	else if (key == 2) {
	increaseValue(tmp);}

	else if (key == 3) {
	decreaseValue(tmp);}

		/* Printing the screen value of the desired element (based on screenvalue struct) */
		printValue(tmp);

		/* Printing the average values on screen */
		setCursorPosLCD(0, 0);
		writeIntegerLCD(averageAdc3, DEC);

		setCursorPosLCD(1, 0);
		writeIntegerLCD(averageAdc2, DEC);

		mSleep(50);

/**********************************************************************
* The following code block is the implementation of PID.. For more information
* about the theory behind it, please consult the attached report.
**********************************************************************/

	/* Measuring two error values, an error value for each sensor */
	error = desiredDistance - averageAdc3;
	error2 = desiredDistance - averageAdc2;

	/* Measuring the sum error for KI. Since kI is not used here, we commented these two lines */
//	sumError += error;
//	sumError2 += error2;

	/* Measuring the speed correction using the forumula provided in the presentation. Note that kI is taken out */
	speedCorrection = (screenvalues.Kp * error) + (screenvalues.Kd * (error-lastError));
	speedCorrection2 = (screenvalues.Kp * error2) + (screenvalues.Kd * (error2-lastError2));

	/* Noting the last error values for KD */
	lastError = error;
	lastError2 = error2;

	/* To ensure that the correction speed does not go too high or too low */
	if(speedCorrection > screenvalues.speed){
		speedCorrection = screenvalues.speed ;
	}
 	if(speedCorrection < -1 * screenvalues.speed ){
		speedCorrection = -1 * (screenvalues.speed / 2);
	}

/* This code did not work for us, we attempted to change the speed based on the sum value of the correction speed of the two sensors */
//	moveAtSpeed( screenvalues.speed - (speedCorrection3 + speedCorrection2), screenvalues.speed + (speedCorrection3 + speedCorrection2));
	moveAtSpeed( screenvalues.speed - (speedCorrection), screenvalues.speed + (speedCorrection));
} //end while
	return 0;
} //end main