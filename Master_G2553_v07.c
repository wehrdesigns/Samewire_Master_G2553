/*
* * *
The MIT License (MIT)

Copyright (c) <2014> <Nathan A. Wehr>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
* * *

This firmware provides an interface between the samewire network and an RS232 serial port
It uses a software UART to invert the signal returning from the serfs
It handles redundant communication error checking.
It handles immediate A2D measurements on several pins

*/

#include "msp430g2553.h"
#include "stdbool.h"

#define		Bit_time	1667//1548     // 9600 Baud, SMCLK=16MHz (16MHz/9600)=1667
#define		Bit_time_5	833//733      // Time for half a bit.
#define		Bit_time_RX 1667		// reduced Bit_time for interrupt processing time
#define		Bit_time_RX_Initial 1667; // add 10 bits to Bit_time for processing adjustment (Falling edge of Start Bit + processing Time + Bit_time is the center of the first Bit)

#define		TXD		BIT5    // TXD on P1.5
#define		RXD		BIT6    // RXD on P1.6

#define		FWType1			'M'	// Master
#define		FWType0			'C'	// Control Power
#define		Version1		'0'
#define		Version0		'7'
#define		ID				'~'

bool bRXBit;				 	// a bit is being received
bool bRXByte;					// a byte has been received
bool bStopbit;					// capture rising edge of StopBit
unsigned int TXByte;			// Byte for Transmit()
unsigned int RXByte;			// Received byte
unsigned char cBit;				// Counter for transmitting a byte

char CmdBuf[]="                              ";     // Buffer to store received command and parameters
signed char cCmd = -1;	  		// Index for CmdBuf
char SendBuf[]="                                        ";    // Buffer for communications
signed char cSend = -1;			// Index for SendBuf[]

bool ADCDone;					// ADC Done flag
unsigned int ADCValue;			// Measured ADC Value

char *Flash_ptrA = (char *) 0x10C0;                         // Segment A pointer
char *Flash_ptrB = (char *) 0x1080;                         // Segment B pointer
char *Flash_ptrC = (char *) 0x1040;                         // Segment C pointer
char *Flash_ptrD = (char *) 0x1000;                         // Segment D pointer

unsigned long *FlashReadDelay = (unsigned long *) 0x1000;
unsigned long LastReadDelay;
unsigned long MaxDelay = 0;

// Function Definitions
void Transmit(void);
void TransmitDecimal(unsigned int);
void TransmitExtendedDecimal(unsigned char, unsigned int, char);
void ExecuteCommand(void);
void Single_Measure(unsigned int, unsigned char);
void Average_Measure(unsigned int, unsigned char);
bool ProgramFlashInfoSegment(char *ptrDestSeg,char *ptrDestAddr,char *ptrSource,char NumItems);
unsigned long ConvertAdvCmdParameterFloatToHex(char CmdBufOffset, char MultipleOfTen);
void SendOKNO(bool PF);

void main(void)
{
	unsigned long ulCycles;
	signed int i;

	WDTCTL = WDTPW + WDTHOLD;	// Stop WDT

	BCSCTL1 = CALBC1_16MHZ;		// Set range
	DCOCTL = CALDCO_16MHZ;		// SMCLK = DCO = 16MHz
	// Initialize USCI UART
    UCA0CTL1 |= UCSWRST;				// Disable USCI
    UCA0CTL1 = UCSSEL_2 + UCSWRST;		//SMCLK
    //http://www.daycounter.com/Calculators/MSP430-Uart-Calculator.phtml
    //http://e2e.ti.com/support/microcontrollers/msp430/f/166/t/18687.aspx
    // 9600 baud settings for 16Mhz
    UCA0MCTL = UCBRS_6;
    UCA0BR0 = 82;
    UCA0BR1 = 6;
    //9600 baud settings for 1Mhz
//    UCA0MCTL = UCBRF_0 + UCBRS_1;
//	UCA0BR0 = 104;
//	UCA0BR1 = 0;

    UCA0CTL1 &= ~UCSWRST;
    IFG2 &= ~(UCA0RXIFG);
    IE2 |= UCA0RXIE;
    // Select Secondary Peripheral Module for USCI P1.1 RX and P1.2 TX
    P1SEL = BIT1 + BIT2;
    P1SEL2 = BIT1 + BIT2;

//	P1SEL |= TXD;				// Connect TXD to timer pin
	P1DIR |= TXD;				// Set TX pin as an output
	CCTL0 &= ~ CCIE ;			// Disable interrupt
	CCTL0 &= ~OUT;				// Set TXD LOW (inverted)
	CCTL0 &= ~(OUTMOD2 + OUTMOD1 + OUTMOD0);			// Set TXD output only mode
	P1SEL &= ~TXD;				// Connect TXD to IO
	P1OUT &= ~TXD;				// Set TX Pin low (inverted)

	P1DIR |= BIT0;				// P1.0 as output to control high current drive
	P1OUT |= BIT0;				// Initialize high to enable power for serfs

    P1DIR |=   BIT3 + BIT4 + BIT7;				// Ground Pin
    P1OUT &= ~(BIT3 + BIT4 + BIT7);			// Ground Pin

    ADC10AE0 |= (BIT3 + BIT4 + BIT7);

	P1IES &= ~RXD;				// RXD Lo/Hi edge interrupt, INVERT to handle serf inverted drive
	P1IFG &= ~RXD;				// Clear RXD (flag)
	P1IE &= ~RXD;				// Disable RXD interrupt
// Disable interrupt on all Port1 pins except P1.6
	P1IE &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT7);

	bRXBit = false; 			// Set initial values
	bRXByte = false;
	cSend = -1;

	// If the FlashReadDelay is default, then initialize to a smaller value
	if (*FlashReadDelay == 0xFFFFFFFF){
		unsigned long l = 1000; // initialize to this value
		char a[4];
		a[3] = l>>24;
		a[2] = l>>16;
		a[1] = l>>8;
		a[0] = l;
		void *ptr = FlashReadDelay;
		ProgramFlashInfoSegment(Flash_ptrD,ptr,a,4);
	}
	// For Debugging
//	P2DIR |= BIT0 + BIT1 + BIT2 + BIT3;
//	P2OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3);

	__bis_SR_register(GIE); 	// interrupts enabled

	while(1){
		//Check for a Carriage Return and run command if found
		if (CmdBuf[cCmd] == 0x0D){
			IE2 &= ~UCA0RXIE;
			if(CmdBuf[0] == ID){	//Command string must be a specific length (ID-1)(Cmd-2)(:)(Parameters-1or2)(CR-1); remember the first character is cCmd=0
				if(cCmd == 3 || (cCmd > 3 && CmdBuf[3] == ':'))
					ExecuteCommand();
				cCmd=-1;							//Reset Receive byte counter
			}else{									//Wait for a Carriage Return before retransmitting
//				P2OUT |= BIT3;//debug
				P1OUT &= ~BIT0; 		// Disable high current drive
				P1IE &= ~RXD;			// Disable RXD interrupt
				P1IFG &= ~RXD;				// Clear RXD (flag)
				//For Power Line
//				P1OUT &= ~TXD;				// Turn off TXD pin
				CCTL0 &= ~CCIE ;			// Disable interrupt
				CCTL0 &= ~CCIS0;
				CCTL0 &= ~OUT;				// Set TXD LOW (inverted)
				CCTL0 &= ~(OUTMOD2 + OUTMOD1 + OUTMOD0);			// Set TXD to output only mode
				P1SEL |= TXD;				// Connect TXD to timer pin (was being used to power the line)

//				__delay_cycles (300);		// Delay for Transmitter to turn on and Receiver to turn off

				i = 0;
				while (i<=cCmd){
					TXByte = CmdBuf[i++];
					Transmit();
				}

				//Turn off TXD pin
//				CCTL0 &= ~(OUTMOD2 + OUTMOD1 + OUTMOD0 + OUT);
				CCTL0 &= ~CCIS0;			// debug code, this can be removed if it can be verified that it is not needed.  Somehow the CCIS0 bit was getting set
				P1OUT &= ~TXD;				// allow line to go high
				P1SEL &= ~TXD;				// Connect TXD to IO
				RXByte = 0;
				bRXByte = false;
				TACTL = TASSEL_2 + MC_2;	// SMCLK, continuous mode
				P1IES &= ~RXD;				// RXD Lo/Hi edge interrupt, INVERT to handle serf inverted drive
				P1IFG &= ~RXD;				// Clear RXD (flag) before enabling interrupt
				ulCycles = 0;
				// Wait so we don't interpret our TX signal dropping as the Start bit from the remote transmitter
				__delay_cycles (280);	// Delay for Transmitter to turn off and Receiver to turn on
				P1IE |= RXD;				// Enable RXD interrupt

				//Wait for response
				unsigned long ulShortWaitCycles;
				char LastSendIndex = 0;
				while (ulCycles < *FlashReadDelay){
					ulCycles++;
					if (SendBuf[cSend] == 0x0D && !bStopbit){
						LastReadDelay = ulCycles;
						if (ulCycles > MaxDelay)
							MaxDelay = ulCycles;
						ulCycles = *FlashReadDelay;
					}
					if (cSend > LastSendIndex){// when a character arrives, expect the remaining characters to follow quickly, otherwise stop waiting
						LastSendIndex = cSend;
						ulShortWaitCycles = Bit_time<<5;// * 10 * 3;
					}
					ulShortWaitCycles--;
					if (ulShortWaitCycles == 0)
						ulCycles = *FlashReadDelay;
				}
				TACTL = TASSEL_2;		// SMCLK, timer off (for power consumption)
				bRXBit = false;
				P1IE &= ~RXD;			// Disable RXD interrupt

				//If a serf has redundant data turned on, filter it before sending to the controller
				//redundant data is the data sent two times, bounded by character 255 (inside the Address and CR characters) and separated by character 255
				//if only one 255 character is found, send an error to the controller
				bool bFSC = false; // Found special character
				char SC = 31; // Special Character
				if(cSend > -1){
					for(i=0;i<=cSend;i++){
						if (SendBuf[i] == SC)
							bFSC = true;
					}
				}
				bool bERROR = false;
				if (bFSC){
					// check for special character at beginning, end and middle
					char middle = cSend>>1;
					if(SendBuf[1] != SC || SendBuf[cSend-1] != SC || SendBuf[middle] != SC)
						bERROR = true;
					for(i=1;i<middle;i++){
						if (SendBuf[i+1] == SendBuf[i+middle]){
							SendBuf[i] = SendBuf[middle+i];
						}else{
							bERROR = true;
						}
					}
					SendBuf[middle-1] = 0x0D;
					cSend = middle-1;
					if(bERROR){
						SendBuf[1] = 'E';
						SendBuf[2] = 'R';
						SendBuf[3] = 'R';
						SendBuf[4] = 'O';
						SendBuf[5] = 'R';
						SendBuf[6] = 0x0D;
						cSend = 6;
					}
				}

//				__delay_cycles (100);

				// Send to Controller
				if(cSend > -1){
					for(i=0;i<=cSend;i++){
						UCA0TXBUF = SendBuf[i];
						while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
					}
				}
			    cSend = -1;

//				while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				UCA0TXBUF = 0x0A;  //send new line
				while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?

//				P1SEL |= TXD;				// Connect TXD to timer pin
//				CCTL0 |= OUT;				// Set TXD HIGH
//				CCTL0 &= ~(OUTMOD2 + OUTMOD1 + OUTMOD0);			// Set TXD high

				P1SEL &= ~TXD;				// Connect TXD to IO
	//			P1DIR |= TXD;				// Set TX pin as an output
	//			P1OUT &= ~TXD;				// Set Transmitter as Power Line
				P1OUT &= ~TXD;				// Set TX Pin low to allow buss to go high
				P1OUT |= BIT0; 				// Enable high current drive
//				__delay_cycles (10000);
				cCmd=-1;
			}
			IE2 |= UCA0RXIE;
		}
	}
}

void ExecuteCommand(void){
	unsigned int inch;
	char Vref;
	unsigned int i;
	bool PF = true;
	unsigned int low;
	unsigned int high;
	unsigned long l;

	SendBuf[++cSend]=ID;		// First load the Comm ID

	//Send Firmware Version
	if((CmdBuf[1] == 'F') && (CmdBuf[2] == 'V')){
		SendBuf[++cSend]=FWType1;
		SendBuf[++cSend]=FWType0;
		SendBuf[++cSend]=Version1;
		SendBuf[++cSend]=Version0;
	}
	else if((CmdBuf[1] == 'A') && (CmdBuf[2] == 'D') && (CmdBuf[3] == ':')){
		P1OUT |= BIT7;				// Initialize P1.7 High - Used to power the sensors
		_delay_cycles (4095);
		inch = INCH_11;	//Initialize
		Vref = 3;		//Initialize
		if(CmdBuf[4] == 'V')
			inch = INCH_11;
		if(CmdBuf[4] == 'T')
			inch = INCH_10;
		if(CmdBuf[4] == '3')
			inch = INCH_3;
		if(CmdBuf[4] == '4')
			inch = INCH_4;
		if(CmdBuf[4] == '5')
			inch = INCH_5;
		if(CmdBuf[4] == '6')
			inch = INCH_6;
		if(CmdBuf[4] == '7')
			inch = INCH_7;
		if(CmdBuf[5] == '1')
			Vref = 1;
		if(CmdBuf[5] == '2')
			Vref = 2;
		if(CmdBuf[5] == '3')
			Vref = 3;
		Average_Measure(inch, Vref);
	}else if((CmdBuf[1] == 'R') && (CmdBuf[2] == 'D') && (CmdBuf[3] == ':')){ // program Read Delay <accepts decimal>  (for 16MHz, recommend 300000)
		l = ConvertAdvCmdParameterFloatToHex(0,0);
		char a[4];
		a[3] = 0;
		a[2] = l>>16;
		a[1] = l>>8;
		a[0] = l;
		void *ptr = FlashReadDelay;
		PF = ProgramFlashInfoSegment(Flash_ptrD,ptr,a,4);
		SendOKNO(PF);
	}else if((CmdBuf[1] == 'R') && (CmdBuf[2] == 'D')){ // Read Delay
		low = *FlashReadDelay;
		high = (*FlashReadDelay >> 16) & 0X00FF;
		TransmitExtendedDecimal(high,low,0);
	}else if((CmdBuf[1] == 'L') && (CmdBuf[2] == 'D')){ // Last Delay
		low = LastReadDelay;
		high = (LastReadDelay >> 16) & 0X00FF;
		TransmitExtendedDecimal(high,low,0);
	}else if((CmdBuf[1] == 'M') && (CmdBuf[2] == 'D')){ // Max Delay
		low = MaxDelay;
		high = (MaxDelay >> 16) & 0X00FF;
		TransmitExtendedDecimal(high,low,0);
		MaxDelay = 0;
	}else if((CmdBuf[1] == 'R') && (CmdBuf[2] == 'S')){ // Reset Serfs
		P1OUT &= ~BIT0; 		// Disable high current drive
		P1OUT |= TXD;				// Set TX Pin high to drive bus low
		for(i=0;i<5;i++)
			__delay_cycles (16000000);
		P1OUT &= ~TXD;				// Set TX Pin low to allow bus to go high
		P1OUT |= BIT0; 		// Enable high current drive
		SendBuf[++cSend]='D';
		SendBuf[++cSend]='O';
		SendBuf[++cSend]='N';
		SendBuf[++cSend]='E';
	}

	CmdBuf[3] = ' ';
	CmdBuf[4] = ' ';
	CmdBuf[5] = ' ';

	SendBuf[++cSend]=0x0D;

	for(i=0;i<=cSend;i++){
		UCA0TXBUF = SendBuf[i];
		while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
	}
	cSend=-1;				// Reset SendBuf Index pointer
	cCmd=-1;				// reset RX byte counter
}

void Transmit(void)
{
	unsigned int i = 0;
	CCTL0 = CCIS0 + OUTMOD0 + CCIE; //invert
	TXByte |= 0x100;			// Add stop bit to TXByte (which is logical 1)
	TXByte = TXByte << 1;		// Add start bit (which is logical 0)
	cBit = 0xA;					// Load Bit counter, 8 bits + ST/SP
	CCTL0 &= ~OUT;				// TXD Idle as Mark (invert)
	TACTL = TASSEL_2 + MC_2;	// SMCLK, continuous mode
	CCR0 = TAR;					// Initialize compare register
	CCR0 += Bit_time;			// Set time till first bit
	CCTL0 =  CCIS0 + OUTMOD0 + OUTMOD2 + CCIE; 	// Reset signal, initial value, enable interrupts (inverted)
	while ( CCTL0 & CCIE && i++ < 26000); 	// Wait for previous TX completion
}

void Single_Measure(unsigned int chan, unsigned char Reference)
{
	/*Reference: 	3 = 3.3V (VCC)
	 * 				2 = 2.5V
	 * 				1 = 1.5V	*/
	ADC10CTL0 &= ~ENC;				// Disable ADC
	if(Reference == 3)
		ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON + ADC10IE;
	if(Reference == 2)
		ADC10CTL0 = SREF_1 + ADC10SHT_3 + ADC10ON + ADC10IE + REFON + REF2_5V;
	if(Reference == 1)
		ADC10CTL0 = SREF_1 + ADC10SHT_3 + ADC10ON + ADC10IE + REFON;
	ADC10CTL1 = ADC10SSEL_3 + chan;				// Set 'chan', SMCLK
	__delay_cycles (1024);					// Delay to allow Ref to settle
	ADC10CTL0 |= ENC + ADC10SC;             	// Enable and start conversion
}

void Average_Measure(unsigned int chan, unsigned char Reference)
{
	unsigned int ADCSum;			// Accumulator and result for ADC averaging
	unsigned int i;
	unsigned char x;
	ADCSum = 0;
	/*Reference: 	3 = 3.3V (VCC)
	 * 				2 = 2.5V
	 * 				1 = 1.5V	*/
	ADC10CTL0 &= ~ENC;				// Disable ADC
	if(Reference == 3)
		ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON;
	if(Reference == 2)
		ADC10CTL0 = SREF_1 + ADC10SHT_3 + ADC10ON + REFON + REF2_5V;
	if(Reference == 1)
		ADC10CTL0 = SREF_1 + ADC10SHT_3 + ADC10ON + REFON;
	ADC10CTL1 = ADC10SSEL_3 + chan;				// Set 'chan', SMCLK
	if(Reference < 3)
		__delay_cycles (30000);					// Delay to allow Ref to settle
	for(x=1;x<=16;x++){
		ADC10CTL0 &= ~ENC;				// Disable ADC
		ADCDone = false;
		ADC10CTL0 &= ~ADC10IFG;
//		ADC10CTL0 |= ADC10IE;
		ADC10CTL0 |= ENC + ADC10SC;             	// Enable and start conversion
		i=0;
		while(i != 254){
			if((ADC10CTL0 & ADC10IFG)== ADC10IFG){
				ADCSum = ADCSum + ADC10MEM;
				__delay_cycles (512);
				i=253;
			}
			i++;
			__delay_cycles (8);
		}
	}
	TransmitDecimal(ADCSum >> 4);
}

unsigned long ConvertAdvCmdParameterFloatToHex(char CmdBufOffset, char MultipleOfTen){
	//parameter is between CmdBuf[5] and cCmd index
	unsigned char d = 0;
	unsigned long l = 0;
	unsigned long m = 0;
	unsigned char arrN[15];
	unsigned char n = 0;

	signed char i=4;
	i += CmdBufOffset;
	while(i<cCmd && (m<MultipleOfTen || MultipleOfTen==0)){
		if (CmdBuf[i] == '.'){
			d = i++;
		}else{
			if (d > 0){
				arrN[n++] = CmdBuf[i++]-'0';
				m++;
			}else if (d == 0){
				arrN[n++] = CmdBuf[i++]-'0';
			}
		}
	}
	//pad zeros
	while (m<MultipleOfTen){
		arrN[n++] = 0;
		m++;
	}
	l = arrN[n-1];
	m=1;
	for (i=n-2;i>=0;i--){
		l += arrN[i] * 10 * m;
		m *= 10;
	}
	return l;
}

//Convert TXByte to unsigned integer in ASCII and add result to SendBuf
//Code from: http://www.cs.uiowa.edu/~jones/bcd/decimal.html
void TransmitDecimal(unsigned int TXByte)
{
	unsigned char d4;
	unsigned char d3;
	unsigned char d2;
	unsigned char d1;
	unsigned char q;
	unsigned int d0;
	
    d0 = TXByte       & 0xF;
    d1 = (TXByte>>4)  & 0xF;
    d2 = (TXByte>>8)  & 0xF;
    d3 = (TXByte>>12) & 0xF;

    d0 = 6*(d3 + d2 + d1) + d0;
    q = d0 / 10;
    d0 = d0 % 10;

    d1 = q + 9*d3 + 5*d2 + d1;
    q = d1 / 10;
    d1 = d1 % 10;

    d2 = q + 2*d2;
    q = d2 / 10;
    d2 = d2 % 10;

    d3 = q + 4*d3;
    q = d3 / 10;
    d3 = d3 % 10;

    d4 = q;

    if(d4!=0)
    	SendBuf[++cSend]=d4 + '0';
    if(d3!=0 || d4!=0)
    	SendBuf[++cSend]=d3 + '0';
    if(d2!=0 || d4!=0 || d3!=0)
    	SendBuf[++cSend]=d2 + '0';
    if(d1!=0 || d4!=0 || d3!=0 ||d2!=0)
    	SendBuf[++cSend]=d1 + '0';
    SendBuf[++cSend]=d0 + '0';
}

void SendOKNO(bool PF){
	//Pass = true, Fail = false
	if(PF){
		SendBuf[++cSend]='O';
		SendBuf[++cSend]='K';
	}else{
		SendBuf[++cSend]='N';
		SendBuf[++cSend]='O';
	}
}

void TransmitExtendedDecimal(unsigned char Extend, unsigned int TXByte, char DecimalPlaces)
{	// Extended to almost 20 bits (999999 decimal)
	unsigned int d6;
	unsigned int d5;
	unsigned int d4;
	unsigned int d3;
	unsigned int d2;
	unsigned int d1;
	unsigned int q;
	unsigned int d0;

    d0 = TXByte       & 0xF;
    d1 = (TXByte>>4)  & 0xF;
    d2 = (TXByte>>8)  & 0xF;
    d3 = (TXByte>>12) & 0xF;
    d4 = Extend & 0xF;

    d0 = 6*(d4 + d3 + d2 + d1) + d0;
    q = d0 / 10;
    d0 = d0 % 10;

    d1 = q + 3*d4 + 9*d3 + 5*d2 + d1;
    q = d1 / 10;
    d1 = d1 % 10;

    d2 = q + 5*d4 + 2*d2;
    q = d2 / 10;
    d2 = d2 % 10;

    d3 = q + 5*d4 + 4*d3;
    q = d3 / 10;
    d3 = d3 % 10;

    d4 = q + 6*d4;
    q = d4 / 10;
    d4 = d4 % 10;

    d5 = q;
    q = d5 / 10;
    d5 = d5 % 10;

    d6 = q;

    if(DecimalPlaces == 7){
    	SendBuf[++cSend]='0';
    	SendBuf[++cSend]='.';
    }
    if(d6!=0 || DecimalPlaces > 5)
        SendBuf[++cSend]=d6 + '0';								//Millions
    if(DecimalPlaces == 6)
    	SendBuf[++cSend]='.';
    if(d5!=0 || d6!=0 || DecimalPlaces > 4)
        SendBuf[++cSend]=d5 + '0';								//Hundred Thousands
    if(DecimalPlaces == 5)
    	SendBuf[++cSend]='.';
    if(d4!=0 || d6!=0 || d5!=0 || DecimalPlaces > 3)
    	SendBuf[++cSend]=d4 + '0';								//Ten Thousands
    if(DecimalPlaces == 4)
    	SendBuf[++cSend]='.';
    if(d3!=0 || d6!=0 || d5!=0 || d4!=0 || DecimalPlaces > 2)
    	SendBuf[++cSend]=d3 + '0';								//Thousands
    if(DecimalPlaces == 3)
    	SendBuf[++cSend]='.';
    if(d2!=0 || d6!=0 || d5!=0 || d4!=0 || d3!=0 || DecimalPlaces > 1)
    	SendBuf[++cSend]=d2 + '0';								//Hundreds
    if(DecimalPlaces == 2)
    	SendBuf[++cSend]='.';
    if(d1!=0 || d6!=0 || d5!=0 || d4!=0 || d3!=0 ||d2!=0 || DecimalPlaces > 0)
    	SendBuf[++cSend]=d1 + '0';								//Tens
    if(DecimalPlaces == 1)
    	SendBuf[++cSend]='.';
    SendBuf[++cSend]=d0 + '0';									//Ones
}

bool ProgramFlashInfoSegment(char *ptrDestSeg,char *ptrDestAddr,char *ptrSource,char NumItems){
	//The NumItems of the array ptrSource will be written starting at ptrDestAddr in the segment ptrDestSeg
		//ptrSource points to an array of length NumItems
		//ptrDestSeg is the address of the information segment to be written to
		//ptrDestAddr is the starting address in the ptrDestSeg segment where the ptrSource array will be written to
	char arrSeg[64];
	char i;
	char n;
	char *d;
	bool pf = true;

	IE1 &= ~WDTIE;

	// Copy Destination Segment to Ram
	d = ptrDestSeg;
	for (i=0;i<64;i++){
		arrSeg[i] = *d++;
	}
	// Handle LOCKA for Information Segment A
//	if(ptrDestSeg == Flash_ptrA){
//		if((FCTL3 & LOCKA) == LOCKA){
//			FCTL3 = FWKEY + LOCKA;
//		}
//	}
	// Erase Segment
	FCTL1 = FWKEY + ERASE;                    // Set Erase bit
	FCTL2 = FWKEY + FSSEL_2 + FN0 + FN1 + FN2 + FN3 + FN4 + FN5; // set for 16Mhz
	FCTL3 = FWKEY;                            // Clear Lock bit
	*ptrDestSeg = 0;                           // Dummy write to erase Flash segment

	FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation

	// Write Ram back to Destination Segment, but use Source data for NumItems when DestSeg matches DestAddr
	d = ptrDestSeg;
	n=0;
	for (i=0;i<64;i++){
		if (d >= ptrDestAddr && n < NumItems){
			*d++ = *(ptrSource + n++);
		}else{
			*d++ = arrSeg[i];
		}
	}

	FCTL1 = FWKEY;                            // Clear WRT bit
	FCTL3 = FWKEY + LOCK;                     // Set LOCK bit

// Verify data
	d = ptrDestSeg;
	n=0;
	for (i=0;i<64;i++){
		if (d >= ptrDestAddr && n < NumItems){
			if(*d++ != *(ptrSource + n++))
				pf = false;
		}else{
			if(*d++ != arrSeg[i])
				pf = false;
		}
	}

	IE1 |= WDTIE;
	return pf;
}

#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{
	ADCValue = ADC10MEM;			// Saves measured value.
	ADCDone = true;  			// Sets flag for main loop.
//	__bic_SR_register_on_exit(CPUOFF);	// Enable CPU so the main while loop continues
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
	if (bStopbit){  // Capture rising edge of stop bit, save RX Byte and prepare to capture falling edge of the next start bit
		SendBuf[++cSend] = RXByte;
		CCTL0 &= ~ CCIE ;		// Disable interrupt
		bStopbit = false;
		P1IES &= ~RXD;				// RXD Lo/Hi edge interrupt, INVERT to handle serf inverted drive
		P1IFG &= ~RXD;			// clear RXD IFG (interrupt flag)
		P1IE |= RXD;			// enabled RXD interrupt
	}
	else{
		CCR0 = TAR;			// Initialize compare register
		CCR0 += Bit_time_RX_Initial;		// Set time till first bit (adjusted for processing delay)
		P2OUT |= BIT1; //debug
		bRXBit = true;
		P1IE &= ~RXD;			// Disable RXD interrupt
		CCTL0 = OUTMOD_2 + CCIE;		// Disable TX and enable interrupts
		cBit = 0x7;			// Load Bit counter, 8 bits
		bStopbit = false;
	}
}

//#pragma vector=PORT2_VECTOR
//__interrupt void Port_2(void)
//{
//
//}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
{
	if(!bRXBit)
	{
		if ( cBit == 0)			// If all bits TXed
		{
			CCTL0 &= ~CCIE ;		// Disable interrupt
		}
		else
		{
			CCR0 += Bit_time;			// Add Offset to CCR0
			CCTL0 &=  ~OUTMOD2;		// Set TX bit to 1 (inverted)
			if (TXByte & 0x01)
				CCTL0 |= OUTMOD2;		// if it should be 1, set it to 0 (inverted)
			TXByte = TXByte >> 1;
			cBit --;
		}
	}
	else
	{
		if ( cBit == 0)
		{
			if ( (P1IN & RXD) == 0)	// If bit is clear?, INVERT to handle serf inverted drive
				RXByte |= 0x80;		// Set the value in the RXByte
			bStopbit = true;			// Capture Rising Edge of Stop Bit
			P1IES |= RXD;				// RXD Hi/Lo edge interrupt, INVERT to handle serf inverted drive
			P1IFG &= ~RXD;			// clear RXD IFG (interrupt flag)
			P1IE |= RXD;			// enabled RXD interrupt
		}
		else
		{
			CCR0 += Bit_time_RX;			// Add Offset to CCR0
			if ( (P1IN & RXD) == 0)	// If bit is set?, INVERT to handle serf inverted drive
				RXByte |= 0x80;		// Set the value in the RXByte
			RXByte = RXByte >> 1;		// Shift the bits down
			cBit --;
		}
	}
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	CmdBuf[++cCmd] = UCA0RXBUF;
	if (CmdBuf[cCmd] == 0x0D)
		IE2 &= ~UCA0RXIE;
	//Check for overflow and reset Cmd Buffer and counter
	if (cCmd == 30){
		cCmd = -1;
		IE2 |= UCA0RXIE;
	}
}

/* Initialize non-used ISR vectors with a trap function */
#pragma vector=NMI_VECTOR,USCIAB0TX_VECTOR
__interrupt void ISR_trap(void)
{
  // the following will cause an access violation which results in a PUC reset
  WDTCTL = 0;
}

