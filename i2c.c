#define _XTAL_FREQ 4000000
#include <xc.h>
#include <pic16f1508.h>





    /*
    Function: I2CInit
    Return:
    Arguments:
    Description: Initialize I2C in master mode, Sets the required baudrate
    */
    void I2CInit(void)
    {
        TRISA = 0b00100011;
        TRISB = 0b01010000;
        
    SSPCONbits.SSPM=0x08; // I2C Master mode, clock = Fosc/(4 * (SSPADD+1))
     SSPCONbits.SSPEN=1; // enable MSSP port
     SSPADD = 0x09; //figure out which one you can ditch sometime (probably either)
    SSP1ADD = 0x09; // 100KHz
    }
     
    /*
    Function: I2CStart
    Return:
    Arguments:
    Description: Send a start condition on I2C Bus
    */
    void I2CStart()
    {
    PIR1bits.SSP1IF=0; // clear SSP interrupt bit
    SSPCON2bits.SEN=1; // send start bit
     while(!PIR1bits.SSP1IF); // Wait for the SSPIF bit to go back high before we load the data buffer
    }
     
    /*
    Function: I2CStop
    Return:
    Arguments:
    Description: Send a stop condition on I2C Bus
    */
    void I2CStop()
    {
    	PEN = 1;         /* Stop condition enabled */
    	while(PEN);      /* Wait for stop condition to finish */
                         /* PEN automatically cleared by hardware */
    }
     
    /*
    Function: I2CRestart
    Return:
    Arguments:
    Description: Sends a repeated start condition on I2C Bus
    */
    void I2CRestart()
    {
    	RSEN = 1;        /* Repeated start enabled */
    	while(RSEN);     /* wait for condition to finish */
    }
     
    /*
    Function: I2CAck
    Return:
    Arguments:
    Description: Generates acknowledge for a transfer
    */
    void I2CAck()
    {
    	ACKDT = 0;       /* Acknowledge data bit, 0 = ACK */
    	ACKEN = 1;       /* Ack data enabled */
    	while(ACKEN);    /* wait for ack data to send on bus */
    }
     
    /*
    Function: I2CNck
    Return:
    Arguments:
    Description: Generates Not-acknowledge for a transfer
    */
    void I2CNak()
    {
    	ACKDT = 1;       /* Acknowledge data bit, 1 = NAK */
    	ACKEN = 1;       /* Ack data enabled */
    	while(ACKEN);    /* wait for ack data to send on bus */
    }
     
    /*
    Function: I2CWait
    Return:
    Arguments:
    Description: wait for transfer to finish
    */
    void I2CWait()
    {
    	while ((SSPCON & 0x1F ) || ( SSP1STAT & 0x04 ) );
        /* wait for any pending transfer */
    }
     
    /*
    Function: I2CSend
    Return:
    Arguments: dat - 8-bit data to be sent on bus
               data can be either address/data byte
    Description: Send 8-bit data on I2C bus
    */
    void I2CSend(unsigned char dat)
    {
    	SSP1BUF = dat;    /* Move data to SSPBUF */
    	while(SSP1STATbits.BF | SSPSTATbits.R_nW);       /* wait till complete data is sent from buffer */
    //	I2CWait();       /* wait for any pending transfer */
    }
     
    /*
    Function: I2CRead
    Return:    8-bit data read from I2C bus
    Arguments:
    Description: read 8-bit data from I2C bus
    */
    unsigned char I2CRead(void)
    {
    	unsigned char temp;
    /* Reception works if transfer is initiated in read mode */
    	RCEN = 1;        /* Enable data reception */
    	while(!BF);      /* wait for buffer full */
    	temp = SSPBUF;   /* Read serial buffer and store in temp register */
    	I2CWait();       /* wait to check any pending transfer */
    	return temp;     /* Return the read data from bus */
    }