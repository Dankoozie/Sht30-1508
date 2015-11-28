/*
 * File:   main.c
 * Author: daniel
 *
 * Created on 07 November 2015, 01:21
 */
#define _XTAL_FREQ 8000000


#include <xc.h>
#include <pic16f1508.h>
#include <stdlib.h>
#include "sht3x.h"
#include "i2c.h"

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF    // Watchdog Timer Enable (WDT enabled while running and disabled in Sleep)
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





//#define _XTAL_FREQ 4000000 //defined for delay
#define device_address 0b1001000 // TCN75A Address (A012 =0)




typedef struct sht_return
{
    unsigned int raw_t;
    unsigned int raw_h;
    char crc_temp;
    char crc_hum;
    char setts;
    char crc_ok;
    
} Sht_rtn;




 
 
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
    //Current setting = 1200bps
    //9600 - TXSTAbits.BRGH=1,SPBRGL=25

// calculate values of SPBRGL and SPBRGH based on the desired baud rate
 //
 // For 8 bit Async mode with BRGH=0: Desired Baud rate = Fosc/64([SPBRGH:SPBRGL]+1)
 // For 8 bit Async mode with BRGH=1: Desired Baud rate = Fosc/16([SPBRGH:SPBRGL]+1)

 TXSTAbits.BRGH=0; // select low speed Baud Rate (see baud rate calcs below)
 TXSTAbits.TX9=0; // select 8 data bits
 TXSTAbits.TXEN = 1; // enable transmit

 RCSTAbits.SPEN=1; // serial port is enabled
 RCSTAbits.RX9=0; // select 8 data bits
 RCSTAbits.CREN=1; // receive enabled

SPBRGL=103; // here is calculated value of SPBRGH and SPBRGL
 SPBRGH=0;

PIR1bits.RCIF=0; // make sure receive interrupt flag is clear
 PIE1bits.RCIE=1; // enable UART Receive interrupt
 INTCONbits.PEIE = 1; // Enable peripheral interrupt
 INTCONbits.GIE = 1; // enable global interrupt

}






unsigned int GetStatus() {
    unsigned char bh,bl;
    unsigned char crc_gen = 0xFF;
    unsigned char crc_rcvd;

I2C_Cmd(0xF3,0x2D);
I2C_restart(); //restart
I2C_Control_Read();
 
 //Read temp high and low bits, crc
 bh = RX_I2C_Data(); //read high
 I2C_ACK(); //ACK
 doCRC(bh,&crc_gen);
 bl = RX_I2C_Data(); //read low
 I2C_ACK(); //ACK
 doCRC(bl,&crc_gen); 

  
 //Read and check temperature CRC 
 crc_rcvd = RX_I2C_Data(); //read low
 I2C_ACK(); //ACK
  
 I2C_NAK(); //NAK
 //I2C_restart();
 I2C_Stop_Bit(); // Send Stop Bit

 
 return (bh<<8) + bl;
 
}




Sht_rtn GetReading() {
    Sht_rtn local;
    unsigned char bh,bl;
    unsigned char crc_gen = 0xFF;
    
 //Send command
 I2C_Cmd(CommandB1,CommandB2);

 __delay_ms(15); //wait for conversion
 I2C_restart(); //restart
I2C_Control_Read();
 
 //Read temp high and low bits, crc
 bh = RX_I2C_Data(); //read high
 I2C_ACK(); //ACK
 doCRC(bh,&crc_gen);
 bl = RX_I2C_Data(); //read low
 I2C_ACK(); //ACK
 doCRC(bl,&crc_gen); 

  
 //Read and check temperature CRC 
 local.crc_temp = RX_I2C_Data(); //read low
 I2C_ACK(); //ACK
 local.raw_t = (bh<<8) + bl;
 if(local.crc_temp == crc_gen) local.crc_ok = 1;
 
 crc_gen = 0xFF;
 
 //Read humidity high and low bits
 bh = RX_I2C_Data(); //read low
 I2C_ACK(); //ACK
 doCRC(bh,&crc_gen);
 bl = RX_I2C_Data(); //read low
 doCRC(bl,&crc_gen);
 I2C_ACK(); //ACK
 local.crc_hum = RX_I2C_Data(); //read low
 local.raw_h = (bh<<8) + bl;
 I2C_NAK(); //NAK
 //I2C_restart();
 I2C_Stop_Bit(); // Send Stop Bit

 if(local.crc_hum == crc_gen) local.crc_ok |= 2;
 
 
 
 
 return local;
 
}





void UART_String(char* letters) {
    int i = 0;
    while(letters[i] != 0) {
        uart_xmit(letters[i++]);
    }
}


void zero_b(char bt){
char lp;
uart_xmit(48);
uart_xmit(98);
for(lp=8;lp>0;lp--){
if((bt & (1<<lp-1)) != 0){
uart_xmit(49);
}
else{uart_xmit(48);}
}
}

void clear_status(){
   I2C_Cmd(0x30,0x41); 
}

void Heater(char status){
     if(status == 1) I2C_Cmd(0x30,0x6D);
     else I2C_Cmd(0x30,0x66);
}

int main(void) {
    Sht_rtn received;
    unsigned int gah;
    char buf[9];
    
 
OSCCONbits.IRCF = 0x0e; //set OSCCON IRCF bits to select OSC frequency 8MHz
 OSCCONbits.SCS = 0x02; 
 OPTION_REGbits.nWPUEN = 0; //enable weak pullups (each pin must be enabled individually)

init_io();
 serial_init();
 I2C_Init();

SSPCONbits.SSPM=0x08; // I2C Master mode, clock = Fosc/(4 * (SSPADD+1))
 SSPCONbits.SSPEN=1; // enable MSSP port
 SSPADD = 0x09; //figure out which one you can ditch sometime (probably either)
 SSP1ADD = 0x09; // 100KHz
 // **************************************************************************************

 
 clear_status();
 
 Heater(0);
 while(1){
 __delay_ms(50); // let everything settle.


 received = GetReading();
 

 
 
 
 UART_String("\x1b[36mTemperature: \x1b[37m");
 itoa(buf,CalcTemp(received.raw_t),10);
 UART_String(buf);
 uart_xmit(32);
 
 UART_String("\x1b[32mHumidity: \x1b[37m");
  itoa(buf,CalcHumid(received.raw_h),10);
 UART_String(buf);
 uart_xmit(32);
 
  UART_String("\x1b[31mStatus: \x1b[37m");
  gah = GetStatus();
  zero_b(gah >> 8);
  uart_xmit(32);
  zero_b(gah & 255);
 
 uart_xmit(32);
 
 
 UART_String("\x1b[35mCRC: \x1b[37m");
 itoa(buf,received.crc_temp,16);
 UART_String(buf);
 uart_xmit(10);
 uart_xmit(13);
 
 
 __delay_ms(5000);
    }
}