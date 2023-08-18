#include <XC.h>
#include <sys/attribs.h>
#include <string.h>

#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz) see figure 8.1 in datasheet for more info
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Secondary Oscillator Enable (Disabled)

#define SYSCLK 40000000L
#define DEF_FREQ 16000L
#define Baud2BRG(desired_baud)      ( (SYSCLK / (16*desired_baud))-1)

#define Button_up (1<<15)
#define Button_left (1<<14)
#define Button_right (1<<13)
#define Button_down (1<<10)


void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
	LATBbits.LATB1 = !LATBbits.LATB1; // Desired frequency on RB6
	LATBbits.LATB2 = !LATBbits.LATB1;
	IFS0CLR=_IFS0_T1IF_MASK; // Clear timer 1 interrupt flag, bit 4 of IFS0
}

void SetupTimer1 (void)
{
	// Explanation here:
	// https://www.youtube.com/watch?v=bu6TTZHnMPY
	__builtin_disable_interrupts();
	PR1 =(SYSCLK/(DEF_FREQ*2L))-1; // since SYSCLK/FREQ = PS*(PR1+1)
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

/* UART2Configure() sets up the UART2 for the most standard and minimal operation
 *  Enable TX and RX lines, 8 data bits, no parity, 1 stop bit, idle when HIGH
 *
 * Input: Desired Baud Rate
 * Output: Actual Baud Rate from baud control register U2BRG after assignment*/
int UART2Configure( int desired_baud)
{
    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(desired_baud); // U2BRG = (FPb / (16*baud)) - 1
    // Calculate actual baud rate
    int actual_baud = SYSCLK / (16 * (U2BRG+1));
    return actual_baud;
}

/* SerialTransmit() transmits a string to the UART2 TX pin MSB first
 *
 * Inputs: *buffer = string to transmit */
int SerialTransmit(const char *buffer)
{
    unsigned int size = strlen(buffer);
    while( size)
    {
        while( U2STAbits.UTXBF);    // wait while TX buffer full
        U2TXREG = *buffer;          // send single character to transmit buffer
        buffer++;                   // transmit next character on following loop
        size--;                     // loop until all characters sent (when size = 0)
    }
 
    while( !U2STAbits.TRMT);        // wait for last transmission to finish
 
    return 0;
}
 
/* SerialReceive() is a blocking function that waits for data on
 *  the UART2 RX buffer and then stores all incoming data into *buffer
 *
 * Note that when a carriage return '\r' is received, a nul character
 *  is appended signifying the strings end
 *
 * Inputs:  *buffer = Character array/pointer to store received data into
 *          max_size = number of bytes allocated to this pointer
 * Outputs: Number of characters received */
unsigned int SerialReceive(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
 
    /* Wait for and store incoming data until either a carriage return is received
     *   or the number of received characters (num_chars) exceeds max_size */
    while(num_char < max_size)
    {
        while( !U2STAbits.URXDA);   // wait until data available in RX buffer
        *buffer = U2RXREG;          // empty contents of RX buffer into *buffer pointer

        while( U2STAbits.UTXBF);    // wait while TX buffer full
        U2TXREG = *buffer;          // echo
 
        // insert nul character to indicate end of string
        if( *buffer == '\r')
        {
            *buffer = '\0';     
            break;
        }
 
        buffer++;
        num_char++;
    }
 
    return num_char;
}

int myAtoi(char *str)
{
    int i, res=0;
    for (i=0; str[i]!='\0'; ++i) res=res*10+(str[i]-'0');
    return res;
}

void PrintNumber(int N, int Base, int digits)
{ 
	char HexDigit[]="0123456789ABCDEF";
	int j;
	#define NBITS 32
	char buff[NBITS+1];
	buff[NBITS]=0;

	j=NBITS-1;
	while ( (N>0) | (digits>0) )
	{
		buff[j--]=HexDigit[N%Base];
		N/=Base;
		if(digits!=0) digits--;
	}
	SerialTransmit(&buff[j+1]);
}

// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}

void waitms(int len)
{
    while(len--) wait_1ms();
}
	
void setDelayBasedOnButton(int *delay) {
    if ((PORTB&(1<<26))) {
        *delay = 1000; // 5ms delay for button 1
    } else if ((PORTB&(1<<25))) {
        *delay = 2000; // 10ms delay for button 2
    } else if ((PORTB&(1<<24))) {
        *delay = 3000; // 15ms delay for button 3
    } else if ((PORTB&(1<<21))) {
        *delay = 4000; // 20ms delay for button 4
    }
}

/* Pinout for DIP28 PIC32MX130:
                                          --------
                                   MCLR -|1     28|- AVDD 
  VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/RA0 -|2     27|- AVSS 
        VREF-/CVREF-/AN1/RPA1/CTED2/RA1 -|3     26|- AN9/C3INA/RPB15/SCK2/CTED6/PMCS1/RB15  up
   PGED1/AN2/C1IND/C2INB/C3IND/RPB0/RB0 -|4     25|- CVREFOUT/AN10/C3INB/RPB14/SCK1/CTED5/PMWR/RB14 right
  PGEC1/AN3/C1INC/C2INA/RPB1/CTED12/RB1 -|5     24|- AN11/RPB13/CTPLS/PMRD/RB13 left
   AN4/C1INB/C2IND/RPB2/SDA2/CTED13/RB2 -|6     23|- AN12/PMD0/RB12
     AN5/C1INA/C2INC/RTCC/RPB3/SCL2/RB3 -|7     22|- PGEC2/TMS/RPB11/PMD1/RB11
                                    VSS -|8     21|- PGED2/RPB10/CTED11/PMD2/RB10   down
                     OSC1/CLKI/RPA2/RA2 -|9     20|- VCAP
                OSC2/CLKO/RPA3/PMA0/RA3 -|10    19|- VSS
                         SOSCI/RPB4/RB4 -|11    18|- TDO/RPB9/SDA1/CTED4/PMD3/RB9
         SOSCO/RPA4/T1CK/CTED9/PMA1/RA4 -|12    17|- TCK/RPB8/SCL1/CTED10/PMD4/RB8
                                    VDD -|13    16|- TDI/RPB7/CTED3/PMD5/INT0/RB7
                    PGED3/RPB5/PMD7/RB5 -|14    15|- PGEC3/RPB6/PMD6/RB6
                                          --------
*/

void main (void)
{
    char buf[128]; // declare receive buffer with max size 128
    unsigned int rx_size;
	int newF = 16103;
	unsigned long reload;
	int delay = 0;
	
	DDPCON = 0;
	CFGCON = 0;
	TRISBbits.TRISB1 = 0;
	LATBbits.LATB1 = 0;	
	TRISBbits.TRISB2 = 0;
	LATBbits.LATB2 = 0;	
	INTCONbits.MVEC = 1;
	
	// Configure button pins as inputs
    /*TRISB |= Button_up | Button_down | Button_left | Button_right;

    // Enable pull-up resistors for button pins
    CNPUB |= Button_up | Button_down | Button_left | Button_right;*/
    
    // Configure button pins as inputs with pull-up resistors
    ANSELB &= ~(1<<15); // Set RB15 as a digital I/O
    TRISB |= (1<<15);   // configure pin RB15 as input
    CNPUB |= (1<<15);   // Enable pull-up resistor for RB15
    
    ANSELB &= ~(1<<14); // Set RB15 as a digital I/O
    TRISB |= (1<<14);   // configure pin RB15 as input
    CNPUB |= (1<<14);   // Enable pull-up resistor for RB15
    
    ANSELB &= ~(1<<13); // Set RB15 as a digital I/O
    TRISB |= (1<<13);   // configure pin RB15 as input
    CNPUB |= (1<<13);   // Enable pull-up resistor for RB15
    
    ANSELB &= ~(1<<10); // Set RB15 as a digital I/O
    TRISB |= (1<<10);   // configure pin RB15 as input
    CNPUB |= (1<<10);   // Enable pull-up resistor for RB15
    
    
	SetupTimer1();

    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4; //SET RX to RB8
    RPB9Rbits.RPB9R = 2; //SET RB9 to TX
 
    UART2Configure(115200); // Configure UART2 for a baud rate of 115200
    U2MODESET = 0x8000; // enable UART2
 
    SerialTransmit("Frequency generator for the PIC32MX130F064B.  Output is RB6 (pin 15).\r\n");
    SerialTransmit("By Jesus Calvino-Fraga (c) 2018.\r\n\r\n");
    
    //int delay = 0;
	int up;
	int down;
	int left;
	int right;
	while (1)
	{
		up = PORTB&(1<<15)?1:0;
		right = PORTB&(1<<14)?1:0;
		left = PORTB&(1<<13)?1:0;
		down = PORTB&(1<<10)?1:0;
		
        if (up == 0){
    	delay = 1000;
    	}
    	if (right == 0){
    	delay = 2000;
    	}
    	if (left == 0){
    	delay = 3000;
    	}
    	if (down == 0){
    	delay = 4000;
    	}
			    reload=(SYSCLK/(newF*2L))-1;
		        SerialTransmit("\r\nDelay: ");
		        PrintNumber(delay, 10, 1);
		        
		        T1CONbits.ON = 0;
		        if (delay > 0) {
		        	waitms(delay);
		        	delay = 0;
		        }

		        PR1=reload;
		        T1CONbits.ON = 1;
        SerialTransmit("\r\n");
	}
}