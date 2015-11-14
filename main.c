/*
 * File:   main.c
 * Author: daniel
 *
 * Created on 07 November 2015, 01:21
 */
#define _XTAL_FREQ 4000000


#include <xc.h>
#include <pic16f1508.h>

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = NSLEEP    // Watchdog Timer Enable (WDT enabled while running and disabled in Sleep)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = ON       // Low-Power Brown Out Reset (Low-Power BOR is enabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)


/* 
 * File: newmain.c
 * Author: Charles M Douvier
 * Contact at: http://iradan.com
 *
 * Created on January 26, 2014, 12:00 PM
 *
 * Target Device:
 * 16F1509 on Tautic 20 pin dev board
 *
 * Project:
 * I2C Testing with the TCN75A
 *
 * Version:
 * 0.1 Start Bit, and Control Byte ... check
 * 0.2 /ACK NAK and Stop ... check!
 * 0.3 works+232
 *
 */


#define _XTAL_FREQ 4000000 //defined for delay
#define device_address 0b1001000 // TCN75A Address (A012 =0)
#define SHT_addr = 0x44;


unsigned int ACK_bit;
 int i;
 unsigned char byte, tempbyte1, tempbyte2;
 unsigned char kirk;
 unsigned char tb1,tb2,kirk2;
 
 
void init_io(void) {

ANSELA = 0x00; // all port A pins are digital I/O
 ANSELB = 0x00; // all port A pins are digital I/O
 ANSELC = 0x00; // all port B pins are digital I/O

 TRISAbits.TRISA0 = 1; // output
 TRISAbits.TRISA1 = 1; // output
 TRISAbits.TRISA2 = 1; // output
 TRISAbits.TRISA3 = 1; // output
 TRISAbits.TRISA4 = 1; // output
 TRISAbits.TRISA5 = 1; // output

TRISBbits.TRISB4 = 1; // RB4 I2C SDA, has to be set as an input
 TRISBbits.TRISB5 = 1; // RB5 = nc
 TRISBbits.TRISB6 = 1; // RB6 I2C SCLK, has to be set as an input
 TRISBbits.TRISB7 = 0; // RS232 TX

TRISCbits.TRISC0 = 0; // output
 TRISCbits.TRISC1 = 0; // output
 TRISCbits.TRISC2 = 0; // output
 TRISCbits.TRISC3 = 0; // output
 TRISCbits.TRISC4 = 0; // output
 TRISCbits.TRISC5 = 0; // output
 TRISCbits.TRISC6 = 0; // input
 TRISCbits.TRISC7 = 0; // input

 LATCbits.LATC0 = 1;
 LATCbits.LATC1 = 0;
 LATCbits.LATC2 = 0;
}

void uart_xmit(unsigned int mydata_byte) {

while(!TXSTAbits.TRMT); // make sure buffer full bit is high before transmitting
 TXREG = mydata_byte; // transmit data
}
void serial_init(void)
{

// calculate values of SPBRGL and SPBRGH based on the desired baud rate
 //
 // For 8 bit Async mode with BRGH=0: Desired Baud rate = Fosc/64([SPBRGH:SPBRGL]+1)
 // For 8 bit Async mode with BRGH=1: Desired Baud rate = Fosc/16([SPBRGH:SPBRGL]+1)

 TXSTAbits.BRGH=1; // select low speed Baud Rate (see baud rate calcs below)
 TXSTAbits.TX9=0; // select 8 data bits
 TXSTAbits.TXEN = 1; // enable transmit

 RCSTAbits.SPEN=1; // serial port is enabled
 RCSTAbits.RX9=0; // select 8 data bits
 RCSTAbits.CREN=1; // receive enabled

SPBRGL=25; // here is calculated value of SPBRGH and SPBRGL
 SPBRGH=0;

PIR1bits.RCIF=0; // make sure receive interrupt flag is clear
 PIE1bits.RCIE=1; // enable UART Receive interrupt
 INTCONbits.PEIE = 1; // Enable peripheral interrupt
 INTCONbits.GIE = 1; // enable global interrupt

__delay_ms(50); // give time for voltage levels on board to settle

 uart_xmit('R'); // transmit some data
 uart_xmit('S');
 uart_xmit('T');

}

void I2C_ACK(void)
{
 PIR1bits.SSP1IF=0; // clear SSP interrupt bit
 SSP1CON2bits.ACKDT=0; // clear the Acknowledge Data Bit - this means we are sending an Acknowledge or 'ACK'
 SSP1CON2bits.ACKEN=1; // set the ACK enable bit to initiate transmission of the ACK bit to the serial eeprom
 while(!PIR1bits.SSP1IF); // Wait for interrupt flag to go high indicating transmission is complete
}

void Send_I2C_Data(unsigned int databyte)
{
 PIR1bits.SSP1IF=0; // clear SSP interrupt bit
 SSPBUF = databyte; // send databyte
 while(!PIR1bits.SSP1IF); // Wait for interrupt flag to go high indicating transmission is complete
}

unsigned char RX_I2C_Data (void)
{

RCEN = 1; // 
 while( RCEN ) continue;
 while( !BF ) continue;
 byte = SSPBUF;
 return byte;
}

void I2C_Control_Write(void)
{
 PIR1bits.SSP1IF=0; // clear SSP interrupt bit
 SSP1BUF = 0x44 << 1; // send the control byte (90 TCN75, EF BMP085)
 while(!PIR1bits.SSP1IF) // Wait for interrupt flag to go high indicating transmission is complete
 {
 i = 1;
 // place to add a breakpoint if needed
 }
 PIR1bits.SSP1IF=0;

}

void I2C_Control_Read(void)
{
 PIR1bits.SSP1IF=0; // clear SSP interrupt bit
 SSP1BUF = (0x44 << 1) + 1; // send the control byte (90 TCN75, EF BMP085)
 while(!PIR1bits.SSP1IF){ // Wait for interrupt flag to go high indicating transmission is complete
     i = 1;
 }
     
     
     
 PIR1bits.SSP1IF=0;
 }

void I2C_Start_Bit(void)
{
 PIR1bits.SSP1IF=0; // clear SSP interrupt bit
 SSPCON2bits.SEN=1; // send start bit
 while(!PIR1bits.SSP1IF) // Wait for the SSPIF bit to go back high before we load the data buffer
 {
 i = 1;
 }
 PIR1bits.SSP1IF=0;
}

void I2C_check_idle()
{
 unsigned char byte1; // R/W status: Is a transfer in progress?
 unsigned char byte2; // Lower 5 bits: Acknowledge Sequence, Receive, STOP, Repeated START, START

do
 {
 byte1 = SSPSTAT & 0x04;
 byte2 = SSPCON2 & 0x1F;
 } while( byte1 | byte2 );
}
/*
 * Send the repeated start message and wait repeated start to finish.
 */
void I2C_restart()
{
 I2C_check_idle();
 RSEN = 1; // Reinitiate start
 while( RSEN ) continue;
}

void I2C_Stop_Bit(void)
{
 PIR1bits.SSP1IF=0; // clear SSP interrupt bit
 SSPCON2bits.PEN=1; // send stop bit
 while(!PIR1bits.SSP1IF)
 {
 i = 1;
 // Wait for interrupt flag to go high indicating transmission is complete
 }
}

void I2C_NAK(void)
{
 PIR1bits.SSP1IF=0; // clear SSP interrupt bit
 SSP1CON2bits.ACKDT=1; // set the Acknowledge Data Bit- this means we are sending a No-Ack or 'NAK'
 SSP1CON2bits.ACKEN=1; // set the ACK enable bit to initiate transmission of the ACK bit to the serial eeprom
 while(!PIR1bits.SSP1IF) // Wait for interrupt flag to go high indicating transmission is complete
 {
 i = 1;
 }
}

int main(void) {

OSCCONbits.IRCF = 0x0d; //set OSCCON IRCF bits to select OSC frequency 4MHz
 OSCCONbits.SCS = 0x02; 
 OPTION_REGbits.nWPUEN = 0; //enable weak pullups (each pin must be enabled individually)

init_io();
 serial_init();

SSPCONbits.SSPM=0x08; // I2C Master mode, clock = Fosc/(4 * (SSPADD+1))
 SSPCONbits.SSPEN=1; // enable MSSP port
 SSPADD = 0x09; //figure out which one you can ditch sometime (probably either)
 SSP1ADD = 0x09; // 100KHz
 // **************************************************************************************

 __delay_ms(50); // let everything settle.

 I2C_Start_Bit(); // send start bit
 I2C_Control_Write(); // send control byte with read set

if (!SSP1CON2bits.ACKSTAT)
 LATCbits.LATC1 = 0; //device /ACked?

Send_I2C_Data(0x2C); //pointer
 Send_I2C_Data(0x06); //1 shot, 12bit res
 I2C_Stop_Bit();

 __delay_ms(15); //wait for conversion

//I2C_Start_Bit(); // send start bit
 //I2C_Control_Write(); // send control byte with read set
 //if (!SSP1CON2bits.ACKSTAT)
 //LATCbits.LATC1 = 0;
 //Send_I2C_Data(0x00); //pointer

I2C_restart(); //restart
 I2C_Control_Read();
 RX_I2C_Data(); //read high
 tempbyte1=byte;
 I2C_ACK(); //ACK
 RX_I2C_Data(); //read low
 tempbyte2=byte;
 
 I2C_ACK(); //ACK
 RX_I2C_Data(); //read low
 kirk=byte;
 
 I2C_ACK(); //ACK
 RX_I2C_Data(); //read low
 tb1=byte;
 
 I2C_ACK(); //ACK
 RX_I2C_Data(); //read low
 tb2=byte;
 
 I2C_ACK(); //ACK
 RX_I2C_Data(); //read low
 kirk2=byte;
 
 
 
 I2C_NAK(); //NAK
 
 //I2C_restart();
 I2C_Stop_Bit(); // Send Stop Bit

 
 
 
uart_xmit(tempbyte1); //send data off raw by UART
 uart_xmit(tempbyte2);
 uart_xmit(tb1);
 uart_xmit(tb2);

 __delay_ms(1); // delay.. just because

while (1) {
 __delay_ms(500);
 LATCbits.LATC0 = 1; //blinky
 __delay_ms(500);
 LATCbits.LATC0 = 0;
 }
 return;
}