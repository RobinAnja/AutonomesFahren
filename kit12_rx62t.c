/***********************************************************************/
/*  Supported Microcontroller:RX62T                                    */
/*  File:                   kit12_rx62t.c                              */
/*  File Contents:          MCU Car Trace Basic Program(RX62T version) */
/*  Version number:         Ver.1.00                                   */
/*  Date:                   2013.09.01                                 */
/*  Copyright:              Renesas Micom Car Rally Secretariat        */
/***********************************************************************/
/*
This program supports the following boards:
* RMC-RX62T board
* Sensor board Ver. 5
* Motor drive board Ver. 5
*
*Data about Car NR 2
*PDF: 185
* W=0,143m (Wheelbase)
* T =0,155m (Tread) (vorder abstand != hintere Abstand)
*
* Notation rules:
*  /* For comments about a block 
* // For comments about one line/ parameterdescriptions
**/



/*======================================*/
/* Include                              */
/*======================================*/
#include "iodefine.h"

/*======================================*/
/* Symbol definitions                   */
/*======================================*/

/* Constant settings */
#define PWM_CYCLE       24575           // Motor PWM period (16ms)     
#define SERVO_CENTER    2038            // Servo center value NR 2 2038 
#define HANDLE_STEP     13              // 1 degree value              
#define MAXIMUM_ANGLE	45				        // This is the maximum angle for NR 2 
#define CURVE_ENTRANCE_MOTOR_POWER	10	// motor power for smoothly driving through the 90° curve 
#define TIME_FOR_SLOW_DOWN_CURVE	300	  // time span in which the car should slow down from the actual motor power to the CURVE_ENTRANCE_MOTOR_POWER 

/* Masked value settings X:masked (disabled) O:not masked (enabled)Maske bedeutet welche Sensorn überhaupt abgefragt werden */
#define MASK2_2         0x66            /* X O O X  X O O X            */
#define MASK2_0         0x60            /* X O O X  X X X X            */
#define MASK0_2         0x06            /* X X X X  X O O X            */
#define MASK3_3         0xe7            /* O O O X  X O O O            */
#define MASK0_3         0x07            /* X X X X  X O O O            */
#define MASK3_0         0xe0            /* O O O X  X X X X            */
#define MASK4_0         0xf0            /* O O O O  X X X X            */
#define MASK0_4         0x0f            /* X X X X  O O O O            */
#define MASK4_4         0xff            /* O O O O  O O O O            */

/*Own MASK declarations */
#define MASK1_0				0x80			/* O X X X  X X X X				*/
#define MASK0_1				0x01			/* X X X X  X X X O				*/
#define MASK2_0_Outer		0xc0			/* O O X X  X X X X				*/
#define MASK0_2_Outer		0x03			/* X X X X  X X O O				*/

#define MASK1_1_Inner		0x24			/* X X 0 X  X 0 X X				*/
#define MASK1_1_Middle		0x42			/* X 0 X X  X X 0 X				*/
#define MASK1_1_Outer		0x81			/* 0 X X X  X X X 0				*/

/*======================================*/
/* Prototype declarations               */
/*======================================*/
void init(void);
void timer(unsigned long timer_set);
unsigned char sensor_inp(unsigned char mask);
unsigned char startbar_get(void);
int check_crossline(void);
int check_crossline_gap(void);
int check_rightline(void);
int check_leftline(void);
unsigned char dipsw_get(void);
unsigned char buttonsw_get(void);
unsigned char pushsw_get(void);
void led_out_m(unsigned char led);
void led_out(unsigned char led);
void motor(int accele_l, int accele_r);
void handle(int angle);
void slowDownMotorPower_linear(int time);

/*======================================*/
/* Global variable declarations         */
/*======================================*/
//speedFactor ignores DIP Settings in motor()
//Maximum is 0,7 for secure driving, due to hardware limits
double speedFactor = 0.7;

//Current Speed of car in m/s
// 2 m/s ist so das maximum was der wagen auf mindesetns 1,5 m beschleunigen kann
double measuredSpeed=1.4;

//Distance beetwen Beginning of First Crossline to End of secound Crossline in mm
//Programm Explantion Manual Page 125
double gapDistance = 90;

//Testtimer
int crankTimer=100;

unsigned long cnt0;
unsigned long cnt1;			// Timer
int pattern;
int actualMotorPower = 100;	// important for 90° curve 

/***********************************************************************/
/* Main program                                                        */
/***********************************************************************/
void main(void)
{
	/* Initialize MCU functions */
	init();

	/* Initialize micom car state */
	handle(0);
	motor(0, 0);

	while (1) {
		switch (pattern) {

			/****************************************************************
			Pattern-related
			 0: wait for switch input
			 1: check if start bar is open
			11: normal trace
			12: check end of large turn to right
			13: check end of large turn to left
			21: processing at 1st cross line
			22: read but ignore 2nd time
			220: check second crossline
			221: check normal line after the crosslines
			222: short break to avoid wrong detection	
			23: trace, crank detection after cross line
			31: left crank clearing processing ? wait until stable
			32: left crank clearing processing ? check end of turn
			41: right crank clearing processing ? wait until stable
			42: right crank clearing processing ? check end of turn
			51: processing at 1st right half line detection
			52: read but ignore 2nd line
			53: trace after right half line detection
			54: right lane change end check
			61: processing at 1st left half line detection
			62: read but ignore 2nd line
			63: trace after left half line detection
			64: left lane change end check
			****************************************************************/

		case 0:

			/* Wait for switch input */
			if (pushsw_get()) {
				pattern = 1;
				cnt1 = 0;
				break;
			}

			/* LED flashing processing     */
			if (cnt1 < 100) {          
				led_out(0x1);
			}
			else if (cnt1 < 200) {
				led_out(0x2);
			}
			else {
				cnt1 = 0;
			}

			break;

		case 1:

			/* Check if start bar is open */
			if (!startbar_get()) {
				/* Start!! */
				led_out(0x0);
				pattern = 11;
				cnt1 = 0;
				break;
			}

			/* LED flashing processing     */
			if (cnt1 < 50) {         
				led_out(0x1);
			}
			else if (cnt1 < 100) {
				led_out(0x2);
			}
			else {
				cnt1 = 0;
			}

			break;

		case 11:
			/* Normal trace */

			/* Cross line check */
			if (check_crossline()) {   
				pattern = 21;
				break;
			}

			/* Right half line detection check */
			if (check_rightline()) {   
				pattern = 51;
				while(1){
					motor(0,0);

					if (cnt1 < 50) {           /* LED flashing processing     */
								led_out(0x1);
							}
							else if (cnt1 < 100) {
								led_out(0x2);
							}
							else {
								cnt1 = 0;
							}
				}
				break;
			}

			 /* Left half line detection check */
			if (check_leftline()) {  
				pattern = 61;
				break;

			}

		//	if(check_not_on_track()){  // break if not on track
		//		motor(0,0);
		//		break;
		//	}


			switch (sensor_inp(MASK3_3)) {

			case 0x00:
				/* Center -> straight */
				handle(0);
				motor(100, 100);
				led_out(0x01);
				break;

				//Right Turn
			case 0x04:
				/* Slight amount left of center -> slight turn to right */
				handle(15);		//Default 15
				motor(80, 80);
				break;

			case 0x06:
				/* Small amount left of center -> small turn to right */
				handle(40);		//Default 30
				motor(60, 40);	// Default 20, 16,75
				break;

			case 0x07:
				/* Medium amount left of center -> medium turn to right */			
				handle(55);		//Default 45
				motor(25, 15);	//Default 12,5 9,5
				pattern = 12;
				break;

			case 0x03:
				/* Large amount left of center -> large turn to right */
				handle(60);
				motor(7.5, 4.75);
				break;

			case 0x20:
				/* Slight amount right of center -> slight turn to left */
				handle(-15);
				motor(80, 80);
				led_out(0x02);
				break;

			case 0x60:
				/* Small amount right of center -> small turn to left */
				handle(-40);
				motor(40, 60);
				break;

			case 0xe0:
				/* Medium amount right of center -> medium turn to left */
				handle(-55);
				motor(15, 25);
				pattern = 13;
				break;

			case 0xc0:
				/* Large amount right of center -> large turn to left */
				handle(-60);
				motor(4.75, 7.5);
				break;

			case 0x1:
				handle(-80);
				motor(4, 7);
				break;

			default:
				break;
			}

		case 12:
			/* Check end of large turn to right */
			if (check_crossline()) {  
				/* Cross line check during large turn */
				pattern = 21;
				break;
			}

			if (check_rightline()) {   
				/* Right half line detection check */
				pattern = 51;
				break;
			}

			if (check_leftline()) {    
				/* Left half line detection check */
				pattern = 61;
				break;
			}

			if (sensor_inp(MASK3_3) == 0x06) {
				pattern = 11;
				break;
			}		

		case 13:

			/* Check end of large turn to left */
			if (check_crossline()) {   
				/* Cross line check during large turn */
				pattern = 21;
				break;
			}

			if (check_rightline()) {   
				/* Right half line detection check */
				pattern = 51;
				break;
			}

			if (check_leftline()) {    
				/* Left half line detection check */
				pattern = 61;
				break;
			}

			if (sensor_inp(MASK3_3) == 0x60) {
				pattern = 11;
				break;
			}
			
		case 21:
			//start Timer
			cnt0=0;
			/* Processing at 1st cross line */
			led_out(0x2); //LED 3
			handle(0);
			// initial break on first line read
			motor(20, 20);
			pattern = 22;
			cnt1 = 0;
			break;
			 */
		case 22:
			/* check if car is in gap beetween lines */
			if (check_crossline_gap()) {
				pattern = 220;

			}
			//Falls er in diesen Fall hängt, soll er nach 500ms die prüfung überspringen
			else if(cnt1>500){
				pattern=23;
				led_out(0x0); // LED aus
			}
			break;

		case 220:
			/* check if gap car was in Gap and if we pass the 2nd Crossline */
			if (check_crossline()) { 
				pattern = 221;

			}
			break;
			
		case 221:		
			/* check if we passed the 2nd crossline, after passing the gap */
			if (check_crossline_gap()) {

				//measurement of Speed
				measuredSpeed = gapDistance/cnt0;
				pattern = 222;
				cnt1 = 0;
				//led_out(0x3);
			}
			break;
			
		case 222:
			if (cnt1 > 50) {
				pattern = 23;
				cnt1 = 0;
				while(1){
					motor(0,0);
					led_out(0x3); //LED 2+3

				}
			}
			break;

		case 23:
			while(1){
				motor(0,0);
			}
			/* Trace, crank detection after cross line
			 *
			 * 1 - reconised Line
			 * 0 - not recognised track line
			 * X - deactive Mask Value
			 * */
			if ((sensor_inp(MASK3_0) == 0xe0)) { // 111X XXXX				
				/* Left crank determined -> to left crank clearing processing */
				led_out(0x1);	//LED2
				handle(-45);	//standard (10,50)			
				motor(CURVE_ENTRANCE_MOTOR_POWER, 50);
				pattern = 31;
				cnt1 = 0;
				break;
			}

			if ((sensor_inp(MASK0_3) == 0x07)) { // XXXX X111			
				/* Right crank determined -> to right crank clearing processing */
				handle(45);
				motor(50, CURVE_ENTRANCE_MOTOR_POWER);
				pattern = 41;
				cnt1 = 0;
				break;
			}

			switch (sensor_inp(MASK3_3)) {

			case 0x00:
				/* Center -> straight */
				handle(0);
				motor(50, 50);//break hard
				if ((actualMotorPower == 100) && (!(cnt1 == 0))){
					cnt1 == 0;
        }
				slowDownMotorPower_linear(TIME_FOR_SLOW_DOWN_CURVE);
				led_out(0x3);
				motor(actualMotorPower, actualMotorPower);
				break;

			case 0x04:
				handle(actualMotorPower * 0.15);		// 0.15 in relation to 100 and 80 with handle 15 in pattern 11
				motor((actualMotorPower * 0,8), (actualMotorPower * 0,8));
				break;

			case 0x06:
			case 0x07:
			case 0x03:
				/* Left of center -> turn to right */
				handle(8);
				motor(50, 35);
				break;

			case 0x20:
				handle(-(actualMotorPower * 0.15));		/// 0.15 in relation to 100 and 80 with handle 15 in pattern 11
				motor((actualMotorPower * 0,8), (actualMotorPower * 0,8));
				break;

			case 0x60:
			case 0xe0:
			case 0xc0:
				/* Right of center -> turn to left */
				handle(-8);
				motor(35, 50);
				break;

			default:
				break;
			}

		case 31:
			/* Left crank clearing processing ? wait until stable
			 * Right crank patterm 41
			if (cnt1 > 200) {
				pattern = 32;
				cnt1 = 0;
			}

			break;
			*/

			crankTimer=measuredSpeed*110;

			if(cnt1 > crankTimer){
				pattern =32;
				cnt1=0;
			}
			break;


		case 32:
			/* Left crank clearing processing ? check end of turn */
			if (sensor_inp(MASK3_3) == 0x60) {
				led_out(0x0);
				pattern = 11;
				cnt1 = 0;
			}

			break;

		case 41:
			/* Right crank clearing processing ? wait until stable */
			if (cnt1 > 200) {
				pattern = 42;
				cnt1 = 0;
			}

			break;

		case 42:
			/* Right crank clearing processing ? check end of turn */
			if (sensor_inp(MASK3_3) == 0x06) {
				led_out(0x0);
				pattern = 11;
				cnt1 = 0;
			}

			break;

		case 51:
			/* Processing at 1st right half line detection */
			led_out(0x1);	//LED 3
			handle(0);
			motor(0, 0);
			pattern = 52;
			cnt1 = 0;		// Clear Timer
			break;

		case 52: 
			/* wait 100ms and go to next pattern [PDF 142] */
			/* Read but ignore 2nd time */
			if (cnt1 > 100) {	//wait 100ms
				led_out(0x2);	//LED 2
				pattern = 53;
				cnt1 = 0;
			}

			break;

		case 53:
			/* Trace, lane change after right half line detection */
			if (sensor_inp(MASK4_4) == 0x00) {		// if all sensors reveive null			
				handle(15);							//standard 15
				motor(40, 31);
				pattern = 54;
				cnt1 = 0;
				break;
			}

			switch (sensor_inp(MASK3_3)) {

			case 0x00:
				/* Center -> straight */
				handle(0);
				motor(40, 40);
				break;

			case 0x04:
			case 0x06:
			case 0x07:
			case 0x03:
				/* Left of center -> turn to right */
				handle(8);
				motor(40, 35);
				break;

			case 0x20:
			case 0x60:
			case 0xe0:
			case 0xc0:
				/* Right of center -> turn to left */
				handle(-8);
				motor(35, 40);
				break;

			default:
				break;
			}

		case 54:
			/* Right lane change end check 
			standard ..== 0x3c
			Issue #4
			Issue#10	*/
			if ((sensor_inp(MASK4_4) == 0x18)||
			   (sensor_inp(MASK4_4) == 0x0c) ||
			   (sensor_inp(MASK4_4) == 0x8c)) {
				led_out(0x0);
				pattern = 11;
				cnt1 = 0;
			}

			break;

		case 61:
			/* Processing at 1st left half line detection */
			led_out(0x1);
			handle(0);
			motor(0, 0);
			pattern = 62;
			cnt1 = 0;
			break;

		case 62:
			/* Read but ignore 2nd time */
			if (cnt1 > 100) {
				pattern = 63;
				cnt1 = 0;
			}

			break;

		case 63:
			/* Trace, lane change after left half line detection */
			if (sensor_inp(MASK4_4) == 0x00) {
				handle(-15);
				motor(31, 40);
				pattern = 64;
				cnt1 = 0;
				break;
			}

			switch (sensor_inp(MASK3_3)) {

			case 0x00:
				/* Center -> straight */
				handle(0);
				motor(40, 40);
				break;

			case 0x04:
			case 0x06:
			case 0x07:
			case 0x03:
				/* Left of center -> turn to right */
				handle(8);
				motor(40, 35);
				break;

			case 0x20:
			case 0x60:
			case 0xe0:
			case 0xc0:
				/* Right of center -> turn to left */
				handle(-8);
				motor(35, 40);
				break;

			default:
				break;
			}

		case 64:
			/* Left lane change end check */
			/* Standard 0x3c
			 * Issue #4
			 * Issue#10
			 */
			if ((sensor_inp(MASK4_4) == 0x18) ||
				(sensor_inp(MASK4_4) == 0xc0) ||
			   (sensor_inp(MASK4_4) == 0xc8)) {
				led_out(0x0);
				pattern = 11;
				cnt1 = 0;
			}

			break;

		default:
			/* If neither, return to standby state */
			pattern = 0;
			break;
		}
	}
}

/***********************************************************************/
/* RX62T Initialization                                                */
/***********************************************************************/
void init(void) {
	/* System Clock */
	SYSTEM.SCKCR.BIT.ICK = 0;               //12.288*8=98.304MHz
	SYSTEM.SCKCR.BIT.PCK = 1;               //12.288*4=49.152MHz

	/* Port I/O Settings */
	PORT1.DDR.BYTE = 0x03;                  //P10:LED2 in motor drive board
	PORT2.DR.BYTE = 0x08;
	PORT2.DDR.BYTE = 0x1b;                  //P24:SDCARD_CLK(o)
											//P23:SDCARD_DI(o)
											//P22:SDCARD_DO(i)
											//CN:P21-P20
	PORT3.DR.BYTE = 0x01;
	PORT3.DDR.BYTE = 0x0f;                  //CN:P33-P31
											//P30:SDCARD_CS(o)
	//PORT4:input                           //sensor input
	//PORT5:input
	//PORT6:input
	PORT7.DDR.BYTE = 0x7e;                  //P76:LED3 in motor drive board
											//P75:forward reverse signal(right motor)
											//P74:forward reverse signal(left motor)
											//P73:PWM(right motor)
											//P72:PWM(left motor)
											//P71:PWM(servo motor)
											//P70:Push-button in motor drive board
	PORT8.DDR.BYTE = 0x07;                  //CN:P82-P80
	PORT9.DDR.BYTE = 0x7f;                  //CN:P96-P90
	PORTA.DR.BYTE = 0x0f;                  //CN:PA5-PA4
											//PA3:LED3(o)
											//PA2:LED2(o)
											//PA1:LED1(o)
											//PA0:LED0(o)
	PORTA.DDR.BYTE = 0x3f;                  //CN:PA5-PA0
	PORTB.DDR.BYTE = 0xff;                  //CN:PB7-PB0
	PORTD.DDR.BYTE = 0x0f;                  //PD7:TRST#(i)
											//PD5:TDI(i)
											//PD4:TCK(i)
											//PD3:TDO(o)
											//CN:PD2-PD0
	PORTE.DDR.BYTE = 0x1b;                  //PE5:SW(i)
											//CN:PE4-PE0

	/* Compare match timer */
	MSTP_CMT0 = 0;                          //CMT Release module stop state
	MSTP_CMT2 = 0;                          //CMT Release module stop state

	ICU.IPR[0x04].BYTE = 0x0f;             //CMT0_CMI0 Priority of interrupts
	ICU.IER[0x03].BIT.IEN4 = 1;             //CMT0_CMI0 Permission for interrupt
	CMT.CMSTR0.WORD = 0x0000;				//CMT0,CMT1 Stop counting
	CMT0.CMCR.WORD = 0x00C3;				//PCLK/512
	CMT0.CMCNT = 0;
	CMT0.CMCOR = 96;						//1ms/(1/(49.152MHz/512))
	CMT.CMSTR0.WORD = 0x0003;				//CMT0,CMT1 Start counting

	/* MTU3_3 MTU3_4 PWM mode synchronized by RESET */
	MSTP_MTU = 0;							//Release module stop state
	MTU.TSTRA.BYTE = 0x00;					//MTU Stop counting

	MTU3.TCR.BYTE = 0x23;					//ILCK/64(651.04ns)
	MTU3.TCNT = MTU4.TCNT = 0;              //MTU3,MTU4TCNT clear
	MTU3.TGRA = MTU3.TGRC = PWM_CYCLE;      //cycle(16ms)
	MTU3.TGRB = MTU3.TGRD = SERVO_CENTER;   //PWM(servo motor)
	MTU4.TGRA = MTU4.TGRC = 0;              //PWM(left motor)
	MTU4.TGRB = MTU4.TGRD = 0;              //PWM(right motor)
	MTU.TOCR1A.BYTE = 0x40;                 //Selection of output level
	MTU3.TMDR1.BYTE = 0x38;                 //TGRC,TGRD buffer function
											//PWM mode synchronized by RESET
	MTU4.TMDR1.BYTE = 0x00;                 //Set 0 to exclude MTU3 effects
	MTU.TOERA.BYTE = 0xc7;                 //MTU3TGRB,MTU4TGRA,MTU4TGRB permission for output
	MTU.TSTRA.BYTE = 0x40;                 //MTU0,MTU3 count function
}

/***********************************************************************/
/* Definition:														   */
/*		Interrupt													   */
/***********************************************************************/
#pragma interrupt Excep_CMT0_CMI0(vect=28)
void Excep_CMT0_CMI0(void) {
	cnt0++;
	cnt1++;
}

/***********************************************************************/
/* Definition:                                                         */
/*		 Timer unit													   */
/* Arguments:						                                   */
/*		timer:  value, 1 = 1 ms									       */
/***********************************************************************/
void timer(unsigned long timer_set) {
	cnt0 = 0;
	while (cnt0 < timer_set);
}

/***********************************************************************/
/* Definition:			                                               */
/*		Sensor state detection                                         */
/* Arguments:					                                       */
/*		masked values												   */
/* Return values:				                                       */
/*		sensor value				                                   */
/***********************************************************************/
unsigned char sensor_inp(unsigned char mask) {
	unsigned char sensor;
	sensor = ~PORT4.PORT.BYTE;
	sensor &= mask;

	return sensor;
}

/***********************************************************************/
/* Definition:			                                               */
/*		Read start bar detection sensor                                */
/* Return values:				                                       */
/*		Sensor value, ON (bar present):1,							   */
/*                    OFF (no bar present):0						   */
/***********************************************************************/
unsigned char startbar_get(void) {
	unsigned char b;
	b = ~PORT4.PORT.BIT.B0 & 0x01;     // Read start bar signal      

	return  b;
}

/***********************************************************************/
/* Definition:			                                               */
/*		Check if still on track in lab                                 */
/* Return values:				                                       */
/*		0: still on track, 1: not on track							   */
/***********************************************************************/
int check_not_on_track(void) {
	unsigned char b;
	int ret;
	ret = 0;
	b = sensor_inp(MASK4_4);

	if (b == 0x00) {
		ret = 1;
	}

	return ret;
}

/***********************************************************************/
/* Definition:			                                               */
/*		Check Gap beetween Crossline								   */
/* Return values:				                                       */
/*		still on crossline, 1: inside the gap						   */
/***********************************************************************/
int check_crossline_gap(void) {
	int ret;
	ret = 0;

	if ((sensor_inp(MASK2_0) == 0x00) ||
		(sensor_inp(MASK0_2) == 0x00)) {
		ret = 1;
	}

	return ret;
}

/***********************************************************************/
/* Definition:			                                               */
/*		Cross line detection processing                                */
/* Return values:				                                       */
/*		0: no cross line, 1: cross line								   */
/***********************************************************************/
int check_crossline(void)
{
	if ((sensor_inp(MASK4_4) == 0xff)||
		(sensor_inp(MASK4_4) == 0x7e)||
		(sensor_inp(MASK4_4) == 0x3c))
		{
		return 1;
	}
	return 0;
}

/***********************************************************************/
/* Definition:			                                               */
/*		Right half line detection processing                           */
/* Return values:				                                       */
/*		0: not detected, 1: detected								   */
/***********************************************************************/
int check_rightline(void)
{
	if (sensor_inp(MASK4_4) == 0x1f) {
		return 1;
	}
	return 0;
}

/***********************************************************************/
/* Definition:			                                               */
/*		Left half line detection processing                            */
/* Return values:				                                       */
/*		0: not detected, 1: detected								   */
/***********************************************************************/
int check_leftline(void)
{

	if (sensor_inp(MASK4_4) == 0xf8) {
		return 1;
	}
	return 0;
}

/***********************************************************************/
/* Definition:			                                               */
/*		DIP switch value read                                          */
/* Return values:				                                       */
/*		Switch value, 0 to 15										   */
/***********************************************************************/
unsigned char dipsw_get(void) {
	unsigned char sw, d0, d1, d2, d3;
	d0 = (PORT6.PORT.BIT.B3 & 0x01);		// P63~P60 read                
	d1 = (PORT6.PORT.BIT.B2 & 0x01) << 1;
	d2 = (PORT6.PORT.BIT.B1 & 0x01) << 2;
	d3 = (PORT6.PORT.BIT.B0 & 0x01) << 3;
	sw = d0 | d1 | d2 | d3;

	return  sw;
}

/***********************************************************************/
/* Definition:			                                               */
/*		Push-button in MCU board value read                            */
/* Return values:				                                       */
/*		Switch value, ON: 1, OFF: 0									   */
/***********************************************************************/
unsigned char buttonsw_get(void) {
	unsigned char sw;
	sw = ~PORTE.PORT.BIT.B5 & 0x01;     // Read ports with switches 

	return  sw;
}

/***********************************************************************/
/* Definition:			                                               */
/*		Push-button in motor drive board value read                    */
/* Return values:				                                       */
/*		Switch value, ON: 1, OFF: 0									   */
/***********************************************************************/
unsigned char pushsw_get(void) {
	unsigned char sw;
	sw = ~PORT7.PORT.BIT.B0 & 0x01;    // Read ports with switches   

	return  sw;
}

/***********************************************************************/
/* Definition:			                                               */
/*		LED control in MCU board                                       */
/* Arguments:														   */
/*		Switch value, LED0: bit 0, LED1: bit 1. 0: dark, 1: lit		   */
/***********************************************************************/
void led_out_m(unsigned char led) {
	led = ~led;
	PORTA.DR.BYTE = led & 0x0f;
}

/***********************************************************************/
/* Definition:			                                               */
/*		LED control in motor drive board                               */
/* Arguments:														   */
/*		Switch value, LED0: bit 0, LED1: bit 1. 0: dark, 1: lit		   */
/* Example:															   */
/*		0x3 -> LED1: ON, LED0: ON, 0x2 -> LED1: ON, LED0: OFF		   */
/***********************************************************************/
void led_out(unsigned char led) {
	led = ~led;
	PORT7.DR.BIT.B6 = led & 0x01;
	PORT1.DR.BIT.B0 = (led >> 1) & 0x01;
}

/***********************************************************************/
/* Definition:			                                               */
/*		Motor speed control                                            */
/* Arguments:														   */
/*		Left motor: -100 to 100, Right motor: -100 to 100			   */
/*      Here, 0 is stopped, 100 is forward, and -100 is reverse.	   */
/***********************************************************************/
void motor(int accele_l, int accele_r){
	/* old Settings
	int    sw_data;
	sw_data = dipsw_get() + 5;
	accele_l = accele_l * sw_data / 20;
	accele_r = accele_r * sw_data / 20; */

	/* use speedFactor instead */
	accele_l = accele_l * speedFactor;
	accele_r = accele_r * speedFactor;


	/* Left Motor Control */
	if (accele_l >= 0) {
		PORT7.DR.BYTE &= 0xef;
		MTU4.TGRC = (long)(PWM_CYCLE - 1) * accele_l / 100;
	}
	else {
		PORT7.DR.BYTE |= 0x10;
		MTU4.TGRC = (long)(PWM_CYCLE - 1) * (-accele_l) / 100;
	}

	/* Right Motor Control */
	if (accele_r >= 0) {
		PORT7.DR.BYTE &= 0xdf;
		MTU4.TGRD = (long)(PWM_CYCLE - 1) * accele_r / 100;
	}
	else {
		PORT7.DR.BYTE |= 0x20;
		MTU4.TGRD = (long)(PWM_CYCLE - 1) * (-accele_r) / 100;
	}
}

/***********************************************************************/
/* Definition:			                                               */
/*		Servo steering operation                                       */
/* Arguments:														   */
/*		servo operation angle: -90 to 90							   */
/*      -90: 90-degree turn to left, 0: straight,					   */
/*      90: 90-degree turn to right									   */
/*		Limited to -45 to 45										   */
/***********************************************************************/
void handle(int angle) {
	/* if used angle is to big */
	if(angle>MAXIMUM_ANGLE)angle = MAXIMUM_ANGLE;
	if(angle<-MAXIMUM_ANGLE)angle = -MAXIMUM_ANGLE;

	/* When the servo move from left to right in reverse, replace "-" with "+". */
	MTU3.TGRD = SERVO_CENTER - angle * HANDLE_STEP;
}


/***********************************************************************/
/* Definition:			                                               */
/*		Slow down motor bevore curve                                   */
/* Arguments:														   */
/*		time intervall to slow down motor							   */
/***********************************************************************/													   */
void slowDownMotorPower_linear(int time) {
	if (cnt1 <= time) {
				int timeBetweenEachSlowDownStep = time/(100-CURVE_ENTRANCE_MOTOR_POWER);	
				int slowDownValue = cnt1/timeBetweenEachSlowDownStep;						
					
				if ((100 - slowDownValue) >= CURVE_ENTRANCE_MOTOR_POWER) {
					actualMotorPower = 100 - slowDownValue;
				}
				else{
					actualMotorPower = CURVE_ENTRANCE_MOTOR_POWER;
				}

				}
}

/***********************************************************************/
/* end of file                                                         */
/***********************************************************************/
