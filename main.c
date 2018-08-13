//TODO list
//-The I2C interface will hang forever in init if no pull ups are connected.. so we can test code at all without MPU6050 connected - add timeouts
//-Move path way point constants to flash as to not us up all the RAM
//move nav state machine to its own file out of WayPoints.c

//--- defines to allow us to test with no switches and no gryo - comment out if hardware exists
//#define FORCE_SWITCH_VALUES 
//#define START_POS_SWITCHES	0x03
//#define PLAY_SWITCHES		0x02
//#define NO_GYRO				

#define F_CPU 16000000UL	//specify Arduino Nano frequency - needed for _delay_mS()
//----- Header files provided by Atmel ----------------------
#include <avr/io.h>
#include <util/delay.h>		//provides _delay_ms()
#include <util/twi.h>
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "math.h"			//provides sin, cos etc.
#include "avr/pgmspace.h"	//provides needed macros to load constants in flash so we don't use up all the RAM
//---- Header files for any C files we have written -----------
#include "serial/serial.h"
#include "timing/timing.h"
#include "wayPoints/wayPoints.h"
#include "I2C/i2cmaster.h"
#include "stateMachine/stateMachine.h"
#include "plays/plays.h"

#define TRUE	1				//makes code easier to read
#define FALSE	0				//-
#define HUMAN_READABLE_OUT 0	//0 is syntax when sending to RoboRio or C# test app; 1 is verbose mode

#define BAUD 38400											//The baud rate that we want to use for the UART serial port
#define BAUD_PRESCALLER (((F_CPU / (BAUD * 16UL))) - 1)		//This calculates the scaler of the CPU clock for the desired baud.

#define	DEGREES_PER_RADIAN 57.2958		//will be used to convert cos and sin math functions that use radians to degrees.

#define GYRO_STATE_NOT_INITIALIZED	0
#define GYRO_STATE_READY			1
#define GYRO_STATE_ERROR			2
char gyroState;

#define MPU6050  0xD0     // (0x68 << 1) I2C slave address

//global variables

volatile char requestSensorUpdate;	//set by timer to cause main to update sensor values
volatile char requestSendRobotDrive;//set by timer to cause main to send command to RoboRio
volatile char requestPositionUpdate;//set by interrupt when wheel sensor pulls line low
volatile char requestNav;			//set by RoboRio (or C# test app) on serial port
extern int16_t navState;					//Instantiated in WayPoints.c
extern char requestWayPointAction;
extern volatile char line_received;			//Instantiated in serial.c - signals main that a new line of data was received
extern volatile char data_in[];				//Instantiated in serial.c - The buffer for said data.

char obuf[128];							 //buffer for printing to serial port
int16_t raw_gyro_z;						 //Most significant byte of gyro register pair read from MPU6050
extern int32_t xPos_mInches,yPos_mInches;//Our calculated 2D position referenced to 0,0 when navigation starts.

extern int16_t wayPointIndex;
extern char wayPointAction;
extern int32_t headingTarget;
extern double distanceDelta;
extern int32_t targetWheelEncoderCount;


//TODO: fix this hack that allows us to reset the statemachine when NAV is called a 2nd time during testing
extern int32_t xPos_mInches;
extern int32_t yPos_mInches;
extern int32_t yPos_mInches;
extern char wayPointAction;
extern char requestWayPointAction;		//stalls navigation state machine until action is sent to RoboRio
extern int16_t navState;
extern int16_t wayPointIndex;
extern int32_t headingTarget;
extern double distanceDelta;
extern int32_t targetWheelEncoderCount;
extern int32_t targetDelayCount;			//used by state navigation state machine to stay in state for delay period




char path;
char play;
char colorAssignment;
char startPosition;

//---- forward function declarations -------
void processCommandFromRoboRio();
char getStartPosition();
char getPlay();
void startNav();

int32_t gyroZ_offset_x1000;//used to zero out the offset scaled by 1000 to avoid int div error
int32_t gyroZ_heading_x1000;//Z axis gyro heading in degrees x 1000
int32_t gyroZ_heading_x1000_Last;//Z axis gyro heading in degrees x 1000
int16_t gyroZ_corrected;

int16_t motorDriveLeft;
int16_t motorDriveRight;

int16_t loopCount;
volatile int32_t iInt0FireCount;
int32_t iInt0FireCount_LastProcessed;
volatile char uartDataReceived;

//_delay_mS needs constant at compile time. This allows us to delay a variable duration
void delay_mS(int16_t duration)
{
	for(int i =0; i<duration/10; i++)
	{
		_delay_ms(10);
	}
}

void errorShow(uint16_t delay1, uint16_t delay2)
{
	//how to use ?
	//Load buffer with sprintf like examples below then call this function
	//The led will flash and the message will be sent to serial port (RoboRio?)
	
	//Example 1
	//	sprintf(obuf, "real bad error\r");
	//	errorShow(50,50) //never returns - flash LED on for 50mS, off for 50mS
	
	//Example 2
	//	sprintf(obuf, "made it here in code \r");//load error string
	//	errorShow(50,200);
	
	//Example 3
	//	sprintf(obuf, "err 1 %d %d\r", some_variable1, some_variable2);
	//	errorShow(50,1000)//flash LED on for 50mS, off for 1000mS
	
	while(1)
	{
		ser_send_string(obuf);
		PORTB |= 0x20;
		delay_mS(delay1);
		
		PORTB &= ~0x20;
		delay_mS(delay2);
	}
}

void MPU6050_writereg(uint8_t reg, uint8_t val)
{
	i2c_start(MPU6050+I2C_WRITE);
	i2c_write(reg);  // go to register e.g. 106 user control
	i2c_write(val);  // set value e.g. to 0100 0000 FIFO enable
	i2c_stop();      // set stop condition = release bus
}

uint16_t MPU6050_readreg(uint8_t reg)
{
	uint16_t raw;
	i2c_start_wait(MPU6050+I2C_WRITE);  // set device address and write mode
	i2c_write(reg);                     // ACCEL_XOUT
	i2c_rep_start(MPU6050+I2C_READ);    // set device address and read mode
	raw = i2c_readAck();                // read one intermediate byte
	raw = (raw<<8) | i2c_readNak();     // read last byte
	i2c_stop();
	return raw;
}

void Init_MPU6050()
{
	unsigned char return_value=0;
	_delay_ms(200);  // Wait for 200 ms.

	return_value = i2c_start(MPU6050+I2C_WRITE);       // set device address and write mode, returns 0 if success
	if (return_value)
	{
		/* failed to issue start condition, possibly no device found */
		i2c_stop();
		sprintf(obuf, "failed to start MPU6050\r");//load error string
		gyroState = GYRO_STATE_ERROR;
	}
	else
	{
		/* issuing start condition ok, device accessible */
		MPU6050_writereg(0x6B, 0x00); // go to register 107 set value to 0000 0000 and wake up sensor
		//MPU6050_writereg(0x19, 0x08); // go to register 25 sample rate divider set value to 0000 1000 for 1000 Hz
		//MPU6050_writereg(0x1C, 0x08); // go to register 28 acceleration configuration set value to 0000 1000 for 4g, normal line tension is 2,7g
		//MPU6050_writereg(0x23, 0xF8); // go to register 35 FIFO enable set value to 1111 1000 for all sensors not slave
		//MPU6050_writereg(0x37, 0x10); // go to register 55 interrupt configuration set value to 0001 0000 for logic level high and read clear
		//MPU6050_writereg(0x38, 0x01); // go to register 56 interrupt enable set value to 0000 0001 data ready creates interrupt
		//MPU6050_writereg(0x6A, 0x40); // go to register 106 user control set value to 0100 0000 FIFO enable
		gyroState = GYRO_STATE_READY;
	}
}

void loadRaw_MPU6050(void)
{
	//raw_acc_x = (int)MPU6050_readreg(0x3B);
	//raw_acc_y = (int)MPU6050_readreg(0x3D);
	//raw_acc_z = (int)MPU6050_readreg(0x3F);
	//
	//raw_gyro_x = (int)MPU6050_readreg(0x43);
	//raw_gyro_y = (int)MPU6050_readreg(0x45);
	raw_gyro_z = (int)MPU6050_readreg(0x47);
	raw_gyro_z = -raw_gyro_z;//make CW rotation positive degrees
	
}
void calibrateGyro()
{
	//MP6050 must be still during the final period before this returns
	int32_t calibrationTime_mS = 2000; //calibrate for 2 seconds (values over 1000 are only slightly better)
	//TODO set this back to 5 or more seconds
	//With 2 counts error from the ideal offset value drift was ~0.6 degrees / minute
	//With 0 counts error from the ideal offset value drift was ~0.25 degrees / minute
	//More accuracy would require reading the LSB from the MPU6050 and correcting int div errors again
	//at 20 seconds, The offset values across 10 trials of booting  was 1 count
	//at 10 seconds, 2 counts
	//at 6  seconds, 3 counts
	//at 3  seconds, 5 counts
	//at 1  second,  7 counts
	//at 500mS, the variance was 15 counts
	int16_t iQ = calibrationTime_mS/100;//low pass filter constant - keep small compared to calibration time/10
	for(int32_t i = 0; i < calibrationTime_mS/10; i++) //sample every 10mS i.e. div by 10
	{
		loadRaw_MPU6050();
		_delay_ms(10);
		gyroZ_offset_x1000 = gyroZ_offset_x1000 - gyroZ_offset_x1000/iQ + (1000*(int32_t)raw_gyro_z)/iQ;//low pass filter
	}
	gyroZ_heading_x1000 = 0;
}

int main(void)
{
	//init globals
	iInt0FireCount=0;
	iInt0FireCount_LastProcessed = 0;
	xPos_mInches = 0;
	yPos_mInches = 0;
	motorDriveLeft = 0;
	motorDriveRight = 0;
	gyroState = 0;
	navState = 0;
	colorAssignment = 255;	//Load out of range value to cause error path to be loaded
							// if color assignment is not sent by RoboRio 
							// and start navigation command is sent.
							
	requestWayPointAction = FALSE;
	requestSensorUpdate = FALSE;
	requestSendRobotDrive = FALSE;
	requestPositionUpdate = FALSE;
	requestNav = FALSE;

	//init ports
	DDRB |= 0x20;	//PB5 output - This is the on board LED on the Nano 
	DDRB &= ~0x07;	//PB0,PB1,PB2 Inputs - These are connected to Nano header J1:11,12,13 labeled D8, D9,D10 - Connected to the play select switches.
	PORTB = 0x07;	//Enable pull up resistors on PB0,PB1,PB2

	DDRC &=~0x03;	//PC0,PC1 Inputs - These are connected to Nano header J2:12,11 labeled A0,A1 - connected to starting position select switches
	PORTC |= 0x03;	//Enable pull up resistors on PC0,PC1
	
	//init other stuff
	init_serial();
	i2c_init();
	init_timing();
	
#ifndef NO_GYRO
	Init_MPU6050();
	calibrateGyro();
#endif
	
	while(FALSE)
	{
		PORTB |= 0x20;
		delay_mS(50);
		
		PORTB &= ~0x20;
		delay_mS(50);
	}

	
	DDRD &= ~(1 << DDD2);		// configure PD2 (INT0 pin) as input
	PORTD |= (1 << PORTD2);		// Enable PD2 internal pull-up resistor
	
	EICRA |= (1 << ISC01);	//set INT0 to trigger on falling edge
	EIMSK |= (1 << INT0);		// Turns on INT0
	
	sei();				//Enable Global Interrupt
	_delay_ms(100);		//todo: fix this hack - clear out initial int0 count
	iInt0FireCount=0;	//-

	while (TRUE)
	{
		if(requestSensorUpdate == TRUE)				//request made periodically by timer interrupt handler
		{
			#ifndef NO_GYRO
			loadRaw_MPU6050();//update global variables from MPU6050
			#endif
			requestSensorUpdate = 0; //clear request
			
			int iDegreesPerPeriodScaler = 2638;//This scales down from full scale so that 90 degrees displays as 9000
			gyroZ_heading_x1000 -= (1000 * (int32_t)raw_gyro_z - gyroZ_offset_x1000)/ iDegreesPerPeriodScaler;
			gyroZ_corrected = raw_gyro_z - gyroZ_offset_x1000/1000;//used just to print for debug
		}
		if(requestNav == TRUE)						//request made periodically by timer interrupt handler
		{
			processNav();
			requestNav = FALSE;
		}
		if(line_received == TRUE)					//request made by serial interrupt handler
		{
			processCommandFromRoboRio();
			line_received = FALSE;
		}
		if(requestSendRobotDrive == TRUE)			//request made periodically by timer interrupt handler
		{
			if(HUMAN_READABLE_OUT == TRUE)
			{
				sprintf(obuf, "heading  %06d  offset %06d   raw %06d  hall count %06d\r", (int)(gyroZ_heading_x1000/10), (int)(gyroZ_offset_x1000/1000),  gyroZ_corrected, (int)iInt0FireCount);
			}
			else
			{
				sprintf(obuf, "L%04d,R%04d,A%d,X%ld,Y%ld,H%ld,C%ld, %d, %d, %ld, %0.0f, %ld, %d, %d, %d, %d\r\n", 
				motorDriveLeft, 
				motorDriveRight, 
				wayPointAction, 
				xPos_mInches,
				yPos_mInches, 
				gyroZ_heading_x1000/10, 
				iInt0FireCount,
				(int)navState,
				(int)wayPointIndex,
				headingTarget,
				distanceDelta,
				targetWheelEncoderCount,
				play,
				path,
				startPosition,
				colorAssignment);
			}
			ser_send_string(obuf);
			requestWayPointAction = FALSE;// clear this request because we sent requested action to RoboRio (or terminal window)
			requestSendRobotDrive = FALSE;// clear this request because we sent motor drive
		}
		if(requestPositionUpdate == TRUE) // request made by IRQ0 interrupt handler- D2 falling edge connected to wheel encoder
		{
			int32_t headingAverage;
			
			//calculate how many interrupts need to be processes.. probably 1 but make sure
			int iNeedToProcessCount = iInt0FireCount - iInt0FireCount_LastProcessed;
			iInt0FireCount_LastProcessed = iInt0FireCount;
			//calculate average between last heading and most recent heading... probably very close and we may have lost samples
			if(gyroZ_heading_x1000_Last == 0)
			{
				gyroZ_heading_x1000_Last = gyroZ_heading_x1000;
			}
			headingAverage = (gyroZ_heading_x1000 + gyroZ_heading_x1000_Last)/2;
			gyroZ_heading_x1000_Last = gyroZ_heading_x1000;
			
			//update position
			double rads = ((double) headingAverage/ DEGREES_PER_RADIAN)/1000;
			xPos_mInches -= (double)iNeedToProcessCount * dInchesPerSensorCount * (1000 * sin(rads));
			yPos_mInches += (double)iNeedToProcessCount * dInchesPerSensorCount * (1000 * cos(rads));
			requestPositionUpdate = FALSE;
		}
	}
}
void processCommandFromRoboRio()
{
	//--------- CAL, Calibrate gyro
	if(data_in[0] == 'C' && data_in[1]=='A' && data_in[2] == 'L')
	{
		calibrateGyro();
		return;
	}
	
	//------------LL,LR, RL or RR - Set color assignments. Third character is a don't care term.
	if(data_in[0] == 'L' && data_in[1]=='L')
	{
		colorAssignment = COLOR_ASSIGNMENT_LL;
		return;
	}
	if(data_in[0] == 'L' && data_in[1]=='R')
	{
		colorAssignment = COLOR_ASSIGNMENT_LR;
		return;
	}
	if(data_in[0] == 'R' && data_in[1]=='L')
	{
		colorAssignment = COLOR_ASSIGNMENT_RL;
		return;
	}
	if(data_in[0] == 'R' && data_in[1]=='R')
	{
		colorAssignment = COLOR_ASSIGNMENT_RR;
		return;
	}
	
	//--------- NAV - start Navigation -----------------------------------------------------------------
	if(data_in[0] == 'N' && data_in[1]=='A' && data_in[2] == 'V')
	{
		gyroZ_heading_x1000 = 0; //zero out any drift that accumulated since we calibrated - no time to calibrate now
		xPos_mInches = 0;//back to start
		yPos_mInches = 0;
		play = getPlay();
		startPosition = getStartPosition();
		path = getPath(play, startPosition, colorAssignment);	//Get the path from plays.
		loadPath(path);											//Point ptrPath at that path.
		startNav(); //Start the NAV state machine (uses pointer ptrPath).
		return;
	} 
}

char getStartPosition()
{
	//TODO: read pin states
	//returns 0, 1,2 or 3 depending on pin inputs. There are only 3 valid start positions: 1,2,3 not 0
	//Return 0 anyway to cause the error path to be loaded if user selected 0,0 for the switches
	#ifdef FORCE_SWITCH_VALUES
		return START_POS_SWITCHES;
	#else
		return PINC & 0x03; //read PC1, PC0 - return starting position switch bits
	#endif
}

char getPlay()
{
	#ifdef FORCE_SWITCH_VALUES
		return PLAY_SWITCHES;
	#else
		return PINB & 0x07;//read PB2,PB1,PB0 - return play selection switch bits
	#endif
} 

void startNav()
{
	//reset everything
	xPos_mInches=0; 
	yPos_mInches=0;
	wayPointAction=0;
	requestWayPointAction=0;
	navState=0;
	wayPointIndex=0;
	headingTarget=0;
	distanceDelta=0;
	targetWheelEncoderCount=0;
	targetDelayCount=0;	
	
	//and start nav
	navState = 1;

}