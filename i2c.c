#include "i2c.h"

unsigned int ACK_bit;
int i;

void I2C_Init(void){
    
 TRISBbits.TRISB4 = 1; // RB4 I2C SDA, has to be set as an input
 TRISBbits.TRISB5 = 1; // RB5 = nc
 TRISBbits.TRISB6 = 1; // RB6 I2C SCLK, has to be set as an input
 TRISBbits.TRISB7 = 0; // RS232 TX
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
 char byt;
RCEN = 1; // 
 while( RCEN ) continue;
 while( !BF ) continue;
 byt = SSPBUF;
 return byt;
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

void I2C_Cmd(char b1, char b2){
     //Send command
 I2C_Start_Bit(); // send start bit
 I2C_Control_Write(); // send control byte with read set

if (!SSP1CON2bits.ACKSTAT)
LATCbits.LATC1 = 0; //device /ACked
Send_I2C_Data(b1); //F32D = status command
Send_I2C_Data(b2); 
 I2C_Stop_Bit();
    
}