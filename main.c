/*************************************************************************
**	Author: thaiphd@gmail.com*
**  	Project: SSS - Smart Solding Station, using Hakko 907 iron
**	and PIC16F87XA.
**	Git: www.github.com/wonbinbk
**	Version:3
**	Date: July 19 2016
**	Todo list:
**		-PID control of heater (if necessary(?) Right now it's doing fine
**		without PID)
**		-Allow to adjust PID coefficients while running
**		-Set a timeout when iron rest (display clock) and
**			turn off or set iron to standby mode after 
**			this timeout.
**		-Allow to adjust this timeout while running (in minutes)
*************************************************************************/

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#define _XTAL_FREQ 20000000	//for _delay_ms()
#define _BAUD 57600
/*
**	Common Anode 7SEGMENT leds
**	RB7 - G			RA0 - DIGIT 1	
**	RB6 - B			RA1 - DIGIT 2
**	RB5 - C 		RA2 - DIGIT 3
**	RB4 - D			RC5 - DIGIT 4
**	RB3 - E	
**	RB2 - F
**	RB1 - A
**	RA4 - DP
**	RB0 - Interrupt on change for Holster detection
**	
**	The semicolon ":" only display when RA4=0
**	
**		  a
**	    	---
**	       f|g |b
**	    	----
**	       e|  |c
**	    	----		
**	    	 d
*/
#pragma config WDTE=OFF, PWRTE=OFF, CP=OFF, BOREN=ON, DEBUG=OFF
#pragma config LVP=OFF, CPD=OFF, WRT=OFF, FOSC=HS

/* Variable declare */
unsigned int Kp;//, Ki, Kd;
unsigned int tar,pre_tar,T;//,pre_T;
int e;//,pre_e,sum_e;
div_t result;
unsigned char d[4],led[4],l,i;	
unsigned char hour, minute, second;
unsigned char led7[15]=	{
				0b10000000,	//0
				0b10011110,	//1
				0b100100,	//2
				0b1100,		//3
				0b11010,	//4
				0b1001000,	//5
				0b1000000,	//6
				0b10011100,	//7
				0b0,		//8
				0b1000,		//9
				0b1111110,	//-
				0b1100000,	//E
				0b10010,	//H
				0b111000,	//deg
				0b1101110,  //equal
			};
unsigned char display_scan,op_mod,iron_resting, calib_mod, time_out;
__eeprom unsigned char rom_adc0, rom_adc100;	//t=100*[(ADC-adc0) / (adc100-adc0)] 
unsigned int adc0, adc100;
void init();
void putch(unsigned char c);
void pwm_update(unsigned int duty_cycle);
void print7(unsigned int num, unsigned char display);
unsigned int read_adc(unsigned char channel);
unsigned int read_raw_adc(unsigned char channel);
void heat_control(unsigned int set_T);
void calibration();
#define E 11
#define H 12
#define dash 10
#define S 5
#define deg 13
#define equal 14
#define temp_adj 4
#define temp_in 3
#define tar_unchanged 0
#define tar_changed 1
#define clock 2
#define calib 3
#define four_dash 4	//While calibrating.
#define hr 0
#define mn 1
#define display_error 4
#define time_to_standby 20 	//in minute
#define time_to_turnoff 30	//in minute
#define normal 0
#define standby 1
#define turnoff 2

/*	Interrupt service routine			*/
void __interrupt myisr(void)
	{
		//Check interrupt flag
		if(TMR0IE && TMR0IF)
		{
			TMR0IF=0;
			PORTA=0x10;
			PORTB=led7[d[l]];
			if(l<3)
			{
				PORTA=1<<l | (((!iron_resting) | (second & 0x01))<<4);
				RC5=0;
			}
			else 
			{
				PORTA=0x10;
				RC5=1;
			}	
			l++;
			if (l==4) l=0;	
			if(display_scan!=0) display_scan-=1;
		}
		if(TMR1IE && TMR1IF)
		{
			TMR1IF=0;
			second++;
			if(second==60)
			{
				second=0;
				minute++;
				time_out++;
				if(minute==60)
				{
					minute=0;
					hour++;
					if(hour==24) hour=0;
				}
			}
			TMR1H=0x80;
			TMR1L=0x00;		
		}
	}

void main (void)
	{
		init();
		/****** Check if it reset*/
		print7(8888, calib);
		__delay_ms(500);
		/*************************/
		if(RC6 || RC7) calibration(); //After power on if SW? pressed then go to calibration mode.
		pwm_update(0);
		PORTB=0xFF;
		RC5=0;
		PORTA=0x10;
		Kp=300;
		//Ki=Kd=0;
		adc0= rom_adc0;
		adc100= rom_adc100;
		adc100= adc100 - adc0;
		while(1)
		{	
			tar=read_adc(temp_adj);
			tar=(tar>>3)*5;	//Convert target to Deg C
			if (tar!=pre_tar)	//Only display if changed
			{	
				display_scan=100;	//to display target 100 times. (pause)
				pre_tar=tar;		
			}
			iron_resting=!RB0;//RB0=0 means iron resting on holder
			if(iron_resting)
			{
				print7(hour*100+minute,clock);//and display a clock
				if(time_to_standby<time_out && time_out<time_to_turnoff) op_mod= standby;
				else if(time_out>=time_to_turnoff) op_mod= turnoff;
				else op_mod= normal;
			}
			else
			{
				if(display_scan>0) 	print7(tar,tar_changed);
				else 			print7(T,tar_unchanged);	
				time_out=0;		//Reset time out
				op_mod= normal;
			}
			switch(op_mod)
			{
				case standby:
					if(tar>200)
						heat_control(200);
					else
				case normal:
					heat_control(tar);
					break;
				case turnoff:
					heat_control(0);
					break;
				default:
					break;
			}
			while(RC7 && !RC6 && !RB0)	//Adjust hour
			{
			 	hour++;
			 	if(hour==24) hour=0;
			 	pwm_update(0);
			 	print7(hour*100+minute,clock);
			 	__delay_ms(200);
			}
			while(RC6 && !RC7 && !RB0)	//Adjust minute
			{
				minute++;
				if(minute==60) minute=0;
				pwm_update(0);
				print7(hour*100+minute,clock);
				__delay_ms(200);
			}
		}		
	}	

void heat_control(unsigned int set_T)
{
/***********************************************
** value after calibrating
** for reference, yours iron would be different
** at 0 	degree ADC read 166
** at 100 	degree ADC read 217
***********************************************/
	long duty = 0;
	T=read_adc(temp_in);
	T=(unsigned int)((T-adc0)*(100f/(float)adc100));//Convert T to degree Celsius
	if (set_T>T)	//Since I only use Kp here, if e<=0 don't bother calculating
	{
		e= set_T - T;
		//sum_e= sum_e + e;
		duty = (long)Kp*e ;//+ (long)Ki*sum_e + (long)Kd*(e-pre_e);
		//pre_e= e;
		if (duty>1023) pwm_update(1023);
		else pwm_update((unsigned int)duty);
	}
	else pwm_update(0);
}

/***************************************************************************************************************************
	This function return the hundredth, the tenth and unit of
	required number to display.
	0 <= num <= 999.
	The real LED scanning will happen in ISR (every 6mS).
***************************************************************************************************************************/
void print7(unsigned int num,unsigned char display)
	{
		TMR0IE=0;
		if(abs(tar-num)<display_error) num=tar;
		result=div(num,100);
		d[2]=result.quot;
		num=result.rem;
		result=div(num,10);
		d[1]=result.quot;
		d[0]=result.rem;
		switch (display)
		{
			case tar_unchanged:	//target unchanged, display current temp
				d[3]=d[2];
				d[2]=d[1];
				d[1]=d[0];
				d[0]=deg;
				break;
			case tar_changed:	//target changed, display new target
				d[3]=equal;	
				break;
			case clock:		//in standby mode, display clock
				result=div(d[2],10);
				d[3]=result.quot;
				d[2]=result.rem;
				break;
			case calib:
				result=div(d[2],10);
				d[3]=result.quot;
				d[2]=result.rem;
				break;
			case four_dash:
				d[3]=d[2]=d[1]=d[0]=dash;
				break;
			default:
				break;
		}
		TMR0IE=1;
	}


/***************************************************************************************************************************
	This function update PWM with duty circle.
	PWM frequency is 15.625kHz -> 64uS period.
	Number 64 was chosen for easy calculation.
***************************************************************************************************************************/
void pwm_update(unsigned int duty_cycle)
	{
		CCPR1L= duty_cycle>>2;
		CCP1X = (duty_cycle && 0x02)>>1;
		CCP1Y = duty_cycle && 0x01;
	}

unsigned int read_adc(unsigned char channel)
{
	unsigned int result_adc=0;
	for(i=0; i<8; i++)
	{
		result_adc+=read_raw_adc(channel);
	}
	result_adc= result_adc>>3;
	return result_adc;
}

/***************************************************************************************************************************
	This function queue channel for ADC
	trigger convertion, wait for ISR to return ADC value v.
***************************************************************************************************************************/
unsigned int read_raw_adc(unsigned char channel)
	{
		unsigned int v=0;
		GIE=0;
		ADCON0bits.CHS = channel;
		__delay_us(20);
		GO_DONE=1;
		while(GO_DONE) continue;
		v=(ADRESH<<8) | ADRESL;
		GIE=1;
		return v;
	}
void calibration()
{
	unsigned char calibrating=1;
	while(RC6 || RC7);	//wait for buttons to be released
	while(calibrating)
	{
		print7(0,four_dash);
		__delay_ms(500);
		if(RC6)
		{
			print7(0, tar_unchanged);
			__delay_ms(500);
			adc0= read_adc(temp_in);
			rom_adc0= adc0;
			print7(adc0, calib);
			__delay_ms(1000);
			while(RC6);
		}
		if(RC7)
		{
			print7(100, tar_unchanged);
			__delay_ms(500);
			adc100= read_adc(temp_in);
			rom_adc100= adc100;
			print7(adc100, calib);
			__delay_ms(1000);
			while(RC7);
		}	
	}
}

/**********************INIT PORT AND MODULES*******************************************************************************/	
void init(void)
	{
	//IO
		TRISA = 0x28;	//RA3 & RA5 input RA0,1,2,4 output.
		TRISB = 0x01;	//RB0 input, the rest output.
		TRISC = 0xC3; 	//RC7,6 buttons input, RC5 output, RC4,3 I2C, RC2 output HEAT_CTRL, RC1,0 input for realtime osc.
	//USART for debugging purpose
	/*
		SYNC = 0;		//Synchronous transmit=1, asynch=0
		BRGH = 1;		//for asynch, 1=high speed, 0=low speed
		TX9 = 0;		//9 bit mode = 1, 8 bit mode=0
		SPEN = 1;		//Serial port enable=1, disable=0
		RX9 = 0;		//same as TX9
		CREN = 1;		//Continuous receive=1, disable=0
		FERR = 0;		//Framming error enable=1, disable=0
		OERR = 0;		//Overun error enable=1, disable=0
		SPBRG = _XTAL_FREQ/(16*_BAUD) - 1;		//~36, 0.55% error
		TXEN = 1;		//Transmit enable =1, disable=0
	*/
	//ADC
		ADCON0bits.ADCS = 0b10;	 	//Tad = 64 Tosc= 3.2us
						//Acquisition time =20us, conversion time = 12*Tad=38.4us
						//Total time for ADC ~ 60us
		ADCS2 = 1;
		ADCON0bits.CHS = 3;		//Default AN3
		ADON = 1;
		ADFM = 1;			//Right justified
		ADCON1bits.PCFG = 0;		
			
	//PWM
		CCP1CONbits.CCP1M=0xC;		//PWM mode
		PR2 = 0XFF;			//PWM freq=19.53kHz
						//PWM resolution = 10 bits.
		CCPR1L=0X00;
		CCP1X = 0;
		CCP1Y = 0;			//Duty=0%
		TMR2ON = 1;		
	//Timer0				//Used for scanning LED display
		T0CS = 0;			//Clock source=internal instruction cycle
		PSA=0;			
		OPTION_REGbits.PS=0b110;	//Prescaler 1:128
		TMR0 = 0;			//Interrupt occured every 255*128*0.2=6528us.	
		TMR0IE=1;		
	//Timer1				//Used for RTC
		T1CONbits.T1CKPS = 0;		//Prescaler 1:1
		T1OSCEN = 1;			//Timer1 Osc enabled, to use external oscillator
		TMR1CS = 1;			//Use external clock
		T1SYNC = 1;			//Do not syncronize with internal clock, to use in Sleep mode
		TMR1H=0x80;		
		TMR1ON = 1;			//Enable Timer 1
		TMR1IE = 1;			//Enable Timer 1 interrupt
	//Interrupt
		GIE=1;
		PEIE=1;
		ADIE=0;
		INTEDG=0;
		INTE=0;
	}

/***************************************************************************************************************************
	This function required by printf()
***************************************************************************************************************************/

void putch(unsigned char c)
	{
		while(!TXIF)
			continue;
		TXREG = c;
	}
		
/**************************************************************************************************************************/
