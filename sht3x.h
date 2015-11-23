/* 
 * File:   sht3x.h
 * Author: daniel
 *
 * Created on 21 November 2015, 18:41
 */

#include <xc.h>

#define CRC_POLY 0x31;
void doCRC(char,char*);
int CalcHumid(unsigned int);
int CalcTemp(unsigned int);