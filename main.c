/*
	Version:2
	Date: 21 April 2016
*/
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#define _XTAL_FREQ 20000000	//for _delay_ms()
#define _BAUD 115200
/*
	Common Anode 7SEGMENT leds
	RB7 - G			RA0 - DIGIT 1	
	RB6 - B			RA1 - DIGIT 2
	RB5 - C 		RA2 - DIGIT 3
	RB4 - D			RC5 - DIGIT 4
	RB3 - E	
	RB2 - F
	RB1 - A
	RA4 - DP
	RB0 - Interrupt on change for Holster detection
	
The semicolon ":" only display when RA2=1 & RA4=0
	
		  a
	    	---
	       f|g |b
	    	----
	       e|  |c
	    	----		
	    	 d
*/
#pragma config WDTE=OFF, PWRTE=OFF, CP=OFF, BOREN=OFF, DEBUG=OFF
#pragma config LVP=OFF, CPD=OFF, WRT=OFF, FOSC=HS

// Variable declare
float Temp;
unsigned int v,duty;
unsigned int tar,pre_tar,T,pre_T;
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
				0b1101110,      //equal
			};
#define E 11
#define H 12
#define dash 10
#define S 5
#define deg 13
#define equal 14
#define temp_adj 4
#define temp_in 3
#define tar_changed 1
#define tar_unchanged 0
unsigned char sen_err,heat_err;
unsigned char display_scan;
void init();
void putch(char c);
void pwm_update(unsigned int duty_cycle);
void print7(unsigned int num, unsigned char display);
unsigned int read_adc(unsigned char channel);
//Interrupt service routine			
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
				PORTA=1<<l;
				__delay_us(10);
				RC5=0;
			}
			else 
			{
				RC5=1;
				__delay_us(10);
				PORTA=0x10;
			}	
			l++;
			if (l==4) l=0;	
			if(display_scan!=0) display_scan-=1;
		}
	}
		
void main (void)
	{
		init();
		pwm_update(0);
		PORTB=0xFF;
		RC5=0;
		PORTA=0x10;
		TRISCbits.TRISC7=1;
		TRISCbits.TRISC6=1;
		while(1)
		{
			for(i=0;i<8;i++)
			{
				tar=tar+read_adc(temp_adj);
			}
			tar=150+(tar>>7)*5;
			if (tar!=pre_tar)	//Only display if changed
			{	
				display_scan=200;
				pre_tar=tar;
			}			
			for(i=0;i<8;i++)
			{
				T=T+read_adc(temp_in);
			}
			T=(((T>>3)-187)*100)>>6;//Convert T to degree Celsius
			if(display_scan!=0) print7(tar,tar_changed);
			else print7(T, tar_unchanged);
			if(tar>T) pwm_update(1023);
			else pwm_update(0);
			__delay_ms(100);
			tar=0;
			T=0;
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
	/*	TXEN = 1;		//Transmit enable =1, disable=0
		SYNC = 0;		//Synchronous transmit=1, asynch=0
		BRGH = 1;		//for asynch, 1=high speed, 0=low speed
		TX9 = 0;		//9 bit mode = 1, 8 bit mode=0
		SPEN = 1;		//Serial port enable=1, disable=0
		RX9 = 0;		//same as TX9
		CREN = 1;		//Continuous receive=1, disable=0
		FERR = 0;		//Framming error enable=1, disable=0
		OERR = 0;		//Overung error enable=1, disable=0
		SPBRG = _XTAL_FREQ/(16*_BAUD) - 1;		//~10
	*/
	//ADC
		ADCON0bits.ADCS = 0b10;	 	//Tad = 64 Tosc= 3.2us
									//Acquisition time =20us, conversion time = 12*Tad=38.4us
									//Total time for ADC ~ 60us
		ADCS2 = 1;
		ADCON0bits.CHS = 3;			//Default AN3
		ADON = 1;
		ADFM = 1;					//Right justified
		ADCON1bits.PCFG = 0;		
			
	//PWM
		CCP1CONbits.CCP1M=0xC;	//PWM mode
		PR2 = 0XFF;		//PWM freq=19.53kHz
						//PWM resolution = 10 bits.
		CCPR1L=0X00;
		CCP1X = 0;
		CCP1Y = 0;		//Duty=0%
		TMR2ON = 1;		
	//Timer0			//Used for scanning LED display
		T0CS = 0;		//Clock source=internal instruction cycle
		PSA=0;			
		OPTION_REGbits.PS=0b110;	//Prescaler 1:128
		TMR0 = 0;		//255*128*0.2=6528us. Interrupt occured every 6.5ms.	
		TMR0IE=1;		
	//Timer1			//Used for RTC
		T1CONbits.T1CKPS = 0;		//Prescaler 1:1
		T1OSCEN = 1;	//Timer1 Osc enabled, to use external oscillator
		TMR1CS = 1;		//Use external clock
		T1SYNC = 1;		//Do not syncronize with internal clock, to use in Sleep mode
		TMR1H=0x80;		
		TMR1ON = 0;		//Enable Timer 1
		TMR1IE = 0;		//enable Timer 1 interrupt
	//Interrupt
		GIE=1;
		PEIE=1;
		ADIE=0;
	}


/**************************************************************************************************************************/
/***************************************************************************************************************************
	This function simply help with printf().
***************************************************************************************************************************/
void putch(char c)
	{
		while(TXIF==0)
			continue;
		TXREG = c;
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


/***************************************************************************************************************************
	This function return the hundredth, the tenth and unit of
	required number to display.
	0 <= num <= 999.
	The real LED scanning will happen in ISR (every 6mS).
***************************************************************************************************************************/
void print7(unsigned int num,unsigned char display)
	{
		if(display==tar_unchanged)
		{
			result=div(num,100);
			d[3]=result.quot;
			num=result.rem;
			result=div(num,10);
			d[2]=result.quot;
			d[1]=result.rem;
			d[0]=deg;
		}
		else
		{
			result=div(num,100);
			d[2]=result.quot;
			num=result.rem;
			result=div(num,10);
			d[1]=result.quot;
			d[0]=result.rem;
			d[3]=equal;
		}
	}


/***************************************************************************************************************************
	This function queue channel for ADC
	trigger convertion, wait for ISR to return ADC value v.
***************************************************************************************************************************/
unsigned int read_adc(unsigned char channel)
	{
		GIE=0;
		ADCON0bits.CHS = channel;
		__delay_us(30);
		GO_DONE=1;
		while(GO_DONE) continue;
		v=(ADRESH<<8) + ADRESL;
		GIE=1;
		return v;
	}
