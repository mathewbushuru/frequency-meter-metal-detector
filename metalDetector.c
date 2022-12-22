// project2.c: Metal Detector using PIC32 MCU

#include <XC.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/attribs.h>
#include <stdbool.h>
 
// Configuration Bits (somehow XC32 takes care of this)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Secondary Oscillator Enable (Disabled)

/* Pinout for DIP28 PIC32MX130:

                                   MCLR (1   28) AVDD 
  VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/RA0 (2   27) AVSS 
        VREF-/CVREF-/AN1/RPA1/CTED2/RA1 (3   26) AN9/C3INA/RPB15/SCK2/CTED6/PMCS1/RB15
   PGED1/AN2/C1IND/C2INB/C3IND/RPB0/RB0 (4   25) CVREFOUT/AN10/C3INB/RPB14/SCK1/CTED5/PMWR/RB14
  PGEC1/AN3/C1INC/C2INA/RPB1/CTED12/RB1 (5   24) AN11/RPB13/CTPLS/PMRD/RB13
   AN4/C1INB/C2IND/RPB2/SDA2/CTED13/RB2 (6   23) AN12/PMD0/RB12
     AN5/C1INA/C2INC/RTCC/RPB3/SCL2/RB3 (7   22) PGEC2/TMS/RPB11/PMD1/RB11
                                    VSS (8   21) PGED2/RPB10/CTED11/PMD2/RB10
                     OSC1/CLKI/RPA2/RA2 (9   20) VCAP
                OSC2/CLKO/RPA3/PMA0/RA3 (10  19) VSS
                         SOSCI/RPB4/RB4 (11  18) TDO/RPB9/SDA1/CTED4/PMD3/RB9
         SOSCO/RPA4/T1CK/CTED9/PMA1/RA4 (12  17) TCK/RPB8/SCL1/CTED10/PMD4/RB8
                                    VDD (13  16) TDI/RPB7/CTED3/PMD5/INT0/RB7
                    PGED3/RPB5/PMD7/RB5 (14  15) PGEC3/RPB6/PMD6/RB6
*/

// SPI Flash Memory connections:
// RA1  (pin 3)   (MISO) -> Pin 5 of 25Q32
// RB1  (pin 5)   (MOSI) -> Pin 2 of 25Q32
// RB14 (pin 25)  (SCLK) -> Pin 6 of 25Q32
// RB0  (pin 4)   (CSn)  -> Pin 1 of 25Q32
// 3.3V: connected to pins 3, 7, and 8
// GND:  connected to pin 4

//LCD defines
#define LCD_RS LATBbits.LATB10 // 21
#define LCD_RW LATBbits.LATB2 //6
#define LCD_E  LATBbits.LATB3 //7

#define LCD_D4 LATAbits.LATA2 //9
#define LCD_D5 LATAbits.LATA3 //10
#define LCD_D6 LATBbits.LATB4 //11
#define LCD_D7 LATAbits.LATA4 //12	

// Flash memory commands
#define WRITE_ENABLE     0x06  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define WRITE_DISABLE    0x04  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define READ_STATUS      0x05  // Address:0 Dummy:0 Num:1 to infinite fMax: 32MHz
#define READ_BYTES       0x03  // Address:3 Dummy:0 Num:1 to infinite fMax: 20MHz
#define READ_SILICON_ID  0xab  // Address:0 Dummy:3 Num:1 to infinite fMax: 32MHz
#define FAST_READ        0x0b  // Address:3 Dummy:1 Num:1 to infinite fMax: 40MHz
#define WRITE_STATUS     0x01  // Address:0 Dummy:0 Num:1 fMax: 25MHz
#define WRITE_BYTES      0x02  // Address:3 Dummy:0 Num:1 to 256 fMax: 25MHz
#define ERASE_ALL        0xc7  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define ERASE_BLOCK      0xd8  // Address:3 Dummy:0 Num:0 fMax: 25MHz
#define READ_DEVICE_ID   0x9f  // Address:0 Dummy:2 Num:1 to infinite fMax: 25MHz

// Defines
#define SYSCLK 40000000L
#define DEF_FREQ 22050L
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)

#define PWM_FREQ    200000L
#define DUTY_CYCLE  50

#define SET_CS LATBbits.LATB0=1
#define CLR_CS LATBbits.LATB0=0
#define CHARS_PER_LINE 16

volatile unsigned long int playcnt=0;
volatile unsigned char play_flag=0;

void Init_LCD_Pins (void)
{
    TRISBbits.TRISB10=0;
    TRISBbits.TRISB2=0;
    TRISBbits.TRISB3=0;
    TRISAbits.TRISA2=0;
    TRISAbits.TRISA3=0;
    TRISBbits.TRISB4=0;
    TRISAbits.TRISA4=0;
}

void LCD_4BIT (void)
{
	LCD_E=0; // Resting state of LCD's enable is zero
	LCD_RW=0; // We are only writing to the LCD in this program
	waitms(20);
	// First make sure the LCD is in 8-bit mode and then change to 4-bit mode
	WriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x32); // Change to 4-bit mode

	// Configure the LCD
	WriteCommand(0x28);
	WriteCommand(0x0c);
	WriteCommand(0x01); // Clear screen command (takes some time)
	waitms(20); // Wait for clear screen command to finsih.
}

//---------------------------------;
// Write data to LCD               ;
//---------------------------------;
void WriteData (unsigned char x) {
 	LCD_RS = 1;
	LCD_byte(x);
	waitms(2);
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

//---------------------------------;
// Write command to LCD            ;
//---------------------------------;
void WriteCommand (unsigned char x) {
	LCD_RS = 0;
	LCD_byte(x);
	waitms(5);
}


// Uses Timer4 to delay <us> microseconds
void Timer4us(unsigned char t) 
{
     T4CON = 0x8000; // enable Timer4, source PBCLK, 1:1 prescaler
    // delay 100us per loop until less than 100us remain
    while( t >= 100)
    {
        t-=100;
        TMR4 = 0;
        while( TMR4 < SYSCLK/10000);
    }
    // delay 10us per loop until less than 10us remain
    while( t >= 10)
    {
        t-=10;
        TMR4 = 0;
        while( TMR4 < SYSCLK/100000);
    }
    // delay 1us per loop until finished
    while( t > 0)
    {
        t--;
        TMR4 = 0;
        while( TMR4 < SYSCLK/1000000);
    }
    // turn off Timer4 so function is self-contained
    T4CONCLR = 0x8000;
}

void LCD_pulse(void){
	LCD_E = 1;
	Timer4us(40);
	LCD_E = 0;
}

void LCD_byte (unsigned char x)
{
    LCD_D7=(x & 0x80)?1:0;
    LCD_D6=(x & 0x40)?1:0;
    LCD_D5=(x & 0x20)?1:0;
    LCD_D4=(x & 0x10)?1:0;
    LCD_pulse();
    Timer4us(40); // Or whatever the name of your us function is
    LCD_D7=(x & 0x08)?1:0;
    LCD_D6=(x & 0x04)?1:0;
    LCD_D5=(x & 0x02)?1:0;
    LCD_D4=(x & 0x01)?1:0;
    LCD_pulse();
}

void LCDprint(char * string, unsigned char line, bool clear)
{
 int j;

 WriteCommand(line==2?0xc0:0x80);
 waitms(5);
 for(j=0; string[j]!=0; j++) WriteData(string[j]);// Write the message
 if(clear) for(; j<CHARS_PER_LINE; j++) WriteData(' '); // Clear the rest of the line
}

void config_SPI(void)
{
	int rData;

	// SDI1 can be assigned to any of these pins (table TABLE 11-1: INPUT PIN SELECTION):
	//0000 = RPA1; 0001 = RPB5; 0010 = RPB1; 0011 = RPB11; 0100 = RPB8
	SDI1Rbits.SDI1R=0b0010; //SET SDI1 to RB1, pin 5 of DIP28
    ANSELB &= ~(1<<1); // Set RB1 as a digital I/O
    TRISB |= (1<<1);   // configure pin RB1 as output
	
	// SDO1 can be configured to any of these pins by writting 0b0011 to the corresponding register.
	// Check TABLE 11-2: OUTPUT PIN SELECTION (assuming the pin exists in the dip28 package): 
	// RPA1, RPB5, RPB1, RPB11, RPB8, RPA8, RPC8, RPA9, RPA2, RPB6, RPA4
	// RPB13, RPB2, RPC6, RPC1, RPC3
	RPA1Rbits.RPA1R=0b0011; // config RA1 (pin 3) for SD01
	
	// SCK1 is assigned to pin 25 and can not be changed, but it MUST be configured as digital I/O
	// because it is configured as analog input by default.
    ANSELB &= ~(1<<14); // Set RB14 as a digital I/O
    TRISB |= (1<<14);   // configure RB14 as output
    
    // CSn is assigned to RB0, pin 4.  Also onfigure as digital output pin.
    ANSELB &= ~(1<<0); // Set RB0 as a digital I/O
	TRISBbits.TRISB0 = 0;
	LATBbits.LATB0 = 1;	

	SPI1CON = 0; // Stops and resets the SPI1.
	rData=SPI1BUF; // clears the receive buffer
	SPI1STATCLR=0x40; // clear the Overflow
	SPI1CON=0x10008120; // SPI ON, 8 bits transfer, SMP=1, Master,  SPI mode unknown (looks like 0,0)
	SPI1BRG=8; // About 2.4MHz clock frequency
}

unsigned char SPIWrite(unsigned char a)
{
	SPI1BUF = a; // write to buffer for TX
	while(SPI1STATbits.SPIRBF==0); // wait for transfer complete
	return SPI1BUF; // read the received value
}

// Initially from here:
// http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/pwm
void Init_pwm (void)
{
    // OC1 can be assigned to PA0, PB3, PB4, PB15, and PB7(in use).
    // Check TABLE 11-2: OUTPUT PIN SELECTION in datasheet.
    // Set OC1 to pin PA0 (pin 2 of DIP 28) with peripheral pin select
    RPA0Rbits.RPA0R = 0x0005;
 
    // Configure standard PWM mode for output compare module 1
    OC1CON = 0x0006; 
 
    // A write to PRy configures the PWM frequency
    // PR = [FPB / (PWM Frequency * TMR Prescale Value)] � 1
    PR2 = (SYSCLK / (PWM_FREQ*1)) - 1;
 
    // A write to OCxRS configures the duty cycle
    // : OCxRS / PRy = duty cycle
    OC1RS = (PR2 + 1) * ((float)DUTY_CYCLE / 100);

 	T2CON = 0x0;
    T2CONSET = 0x8000;      // Enable Timer2, prescaler 1:1
	T2CONbits.TCKPS=0x0; // Set pre-scaler to 1
    OC1CONSET = 0x8000;     // Enable Output Compare Module 1
}

void Set_pwm (unsigned char val)
{
	OC1RS = (PR2 + 1) * ((float)val / 256.0);
}

void SetupTimer1 (void)
{
	// Explanation here:
	// https://www.youtube.com/watch?v=bu6TTZHnMPY
	__builtin_disable_interrupts();
	PR1 =(SYSCLK/DEF_FREQ)-1; // since SYSCLK/FREQ = PS*(PR1+1)
	TMR1 = 0;
	T1CONbits.TCKPS = 0; // Pre-scaler: 1
	T1CONbits.TCS = 0; // Clock source
	T1CONbits.ON = 1;
	IPC1bits.T1IP = 5;
	IPC1bits.T1IS = 0;
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;
	
	INTCONbits.MVEC = 1; //Int multi-vector
	__builtin_enable_interrupts();
}

void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
	unsigned char c;
	
	LATBbits.LATB6 = !LATBbits.LATB6; // Toggle pin RB6 (used to check the right frequency)
	IFS0CLR=_IFS0_T1IF_MASK; // Clear timer 1 interrupt flag, bit 4 of IFS0
	
	if(play_flag!=0)
	{  
		if(playcnt==0)
		{
			SET_CS; // Done playing: Disable 25Q32 SPI flash memory
			play_flag=0;
		}
		else
		{
			c=SPIWrite(0x00);
			Set_pwm(c); // Output value to PWM (used as DAC)
			playcnt--;
		}
	}
}
 
void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    //SET RX to RB8
    RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;     // enable UART2
}

// Needed to by scanf() and gets()
int _mon_getc(int canblock)
{
	char c;
	
    if (canblock)
    {
	    while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
	    c=U2RXREG;
	    if(c=='\r') c='\n'; // When using PUTTY, pressing <Enter> sends '\r'.  Ctrl-J sends '\n'
		return (int)c;
    }
    else
    {
        if (U2STAbits.URXDA) // if data available in RX buffer
        {
		    c=U2RXREG;
		    if(c=='\r') c='\n';
			return (int)c;
        }
        else
        {
            return -1; // no characters to return
        }
    }
}

// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}



#define PIN_PERIOD (PORTB&(1<<5))

// GetPeriod() seems to work fine for frequencies between 200Hz and 700kHz.
long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
	}

    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
	}
	
    _CP0_SET_COUNT(0); // resets the core timer count
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
		}
	}

	return  _CP0_GET_COUNT();
}

void Start_Playback (unsigned long int address, unsigned long int numb)
{
    CLR_CS; // Enable 25Q32 SPI flash memory.
    SPIWrite(READ_BYTES);
    SPIWrite((unsigned char)((address>>16)&0xff));
    SPIWrite((unsigned char)((address>>8)&0xff));
    SPIWrite((unsigned char)(address&0xff));
    playcnt=numb;
    play_flag=1;
}

// Information here:
// http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/1-basic-digital-io-220
void main(void)
{
	long int count;
	float T, f, f2, difference;
    unsigned long start, nbytes;
	volatile unsigned long t=0;

	DDPCON = 0;
	CFGCON = 0;

	TRISBbits.TRISB6 = 0;
	LATBbits.LATB6 = 0;	
	INTCONbits.MVEC = 1;

	Init_LCD_Pins();
	Init_pwm(); // pwm output used to implement DAC
	SetupTimer1(); // The ISR for this timer playsback the sound
    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
    config_SPI(); // Configure hardware SPI module

	

	playcnt=0;
	play_flag=0;
	SET_CS; // Disable 25Q32 SPI flash memory

    ANSELB &= ~(1<<5); // Set RB5 as a digital I/O
    TRISB |= (1<<5);   // configure pin RB5 as input
    CNPUB |= (1<<5);   // Enable pull-up resistor for RB5
 
	waitms(500);
	printf("Period measurement using the core timer free running counter.\r\n"
	      "Connect signal to RB5 (pin 14).\r\n");
    f2 =f;

    while(1) {

		LCD_4BIT();

		LATBbits.LATB6 = 0;
		difference = f-f2;

		count=GetPeriod(100);

		if(count>0) {
			T=(count*2.0)/(SYSCLK*100.0);
			f=1/T;
			printf("f=%fHz, DIFFERENCE =%fHz, Count=%ld        \r", f, f-f2, count);
		}
		else {
			printf("NO SIGNAL                     \r");
		}

		playcnt=0;
		play_flag=0;
		SET_CS; // Disable 25Q32 SPI flash memory

		if (f < 31280 && f >= 31200) { // small ferrous metal
			start = 0x00002d;
			nbytes = 0x006e85;
			// Playback a portion of the stored wav file
			Start_Playback(start, nbytes);
			LATBbits.LATB6 = !LATBbits.LATB6; // Blink led on RB6 (pin 15)
			LCDprint("SFM",15,1); // Display on LCD: SFM (Small Ferrous Metal)
		}
		else if (f > 31320 && f <= 31400) { // small non-ferrous metal
			start = 0x006eb2;
			nbytes = 0x009bee;
			// Playback a portion of the stored wav file
			Start_Playback(start, nbytes);
			LATBbits.LATB6 = !LATBbits.LATB6; // Blink led on RB6 (pin 15)
			LCDprint("SNFM",15,1); // Display on LCD: SFM (Small Non-Ferrous Metal)
		}
		else if (f < 31200) { // large ferrous metal
			start = 0x010aa0;
			nbytes = 0x0084b8;
			// Playback a portion of the stored wav file
			Start_Playback(start, nbytes);
			LATBbits.LATB6 = !LATBbits.LATB6; // Blink led on RB6 (pin 15)
			LCDprint("LFM",15,1); // Display on LCD: SFM (Large Ferrous Metal)
		}
		else if (f > 31400) { // large non-ferrous metal
			start = 0x018f58;
			nbytes = 0x00d96a;
			// Playback a portion of the stored wav file
			Start_Playback(start, nbytes);
			LATBbits.LATB6 = !LATBbits.LATB6; // Blink led on RB6 (pin 15)
			LCDprint("LNFM",15,1); // Display on LCD: SFM (Large Non-Ferrous Metal)
		}

		waitms(4000);
		f2 = f;
    }
}