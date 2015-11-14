/* 
 * File:   i2c.h
 * Author: daniel
 *
 * Created on 07 November 2015, 01:22
 */

#ifndef I2C_H
#define	I2C_H


#endif	/* I2C_H */
void I2CInit(void);
void I2CStart();
void I2CStop();
void I2CRestart();
void I2CAck();
void I2CNak();
void I2CWait();
void I2CSend(unsigned char);
unsigned char I2CRead(void);