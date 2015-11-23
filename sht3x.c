#include "sht3x.h"

void doCRC (unsigned char ch, unsigned char *crc)
{
    int ix;
    unsigned char b7;
    
    for (ix = 0; ix < 8; ix++) {
        b7 = ch ^ *crc;
        *crc <<= 1;
        ch <<= 1;
        if (b7 & 0x80)
            *crc ^= CRC_POLY;
    }
    return;
}


int CalcTemp(unsigned int rcv_val) {
    
    return rcv_val;
}

int CalcHumid(unsigned int rcv_val) {
    float ans = ((float)rcv_val / 65535);
    ans = ans * 100;
    
    return (int) ans;
}