/* 
 * File:   i2c.h
 * Author: daniel
 *
 * Created on 22 November 2015, 22:25
 */
#include <xc.h>

#define SHT_addr  0x44;

char const CommandB1 = 0x2C;
char const CommandB2 = 0x06;


void I2C_Init(void);

void I2C_ACK(void);
void Send_I2C_Data(unsigned int);
void I2C_Cmd(char b1, char b2);
void I2C_Control_Write(void);
unsigned char RX_I2C_Data (void);
void Send_I2C_Data(unsigned int);
void I2C_Control_Read(void);
void I2C_Start_Bit(void);
void I2C_check_idle(void);
void I2C_restart();
void I2C_Stop_Bit(void);
void I2C_NAK(void);
