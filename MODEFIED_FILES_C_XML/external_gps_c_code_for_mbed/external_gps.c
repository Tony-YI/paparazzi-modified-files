/**
 * uBlox UBX Protocol Reader - Wayne Holder
 * Ported to mbed - Michael Shimniok
 *
 * Modefied by YI.
 */
#include "mbed.h"

void sendCmd(unsigned char len, uint8_t data[]);
 
#define MAX_LENGTH 512
 
#define  SYNC1       0xB5
#define  SYNC2       0x62
#define  SOL_MSG     0x06

#define INT_32(X)    *(int32_t *)(&data[X])
#define UINT_32(X)   *(uint32_t *)(&data[X])
#define INT_8(X)     *(int8_t *)(&data[X])
#define UINT_8(X)    *(uint8_t *)(&data[X])
 
unsigned char  state, lstate, code, id, chk1, chk2, ck1, ck2;
unsigned int  length, idx, cnt;
bool gpsReady = false;
bool checkOk = false;
 
unsigned char data[MAX_LENGTH];
 
Serial pc(USBTX, USBRX);
Serial gps(p9, p10);
DigitalOut led1(LED1);//LED for rx
DigitalOut led2(LED2);//LED on when get a valid message from GPS
DigitalOut led3(LED3);//LED on when get a 3D-Fix
 
unsigned char buf[MAX_LENGTH];
int in=0;
int out=0;
 
void enableMsg(unsigned char id, bool enable, int rate=4)//i think the rate here is the message sending rate
{
    if (!enable) rate = 0;
    uint8_t cmdBuf[] = {
        0x06,   // class CFG
        0x01,   // id MSG -> CFG-MSG
        8,      // length, for config message rates
        0x01,   // class,
        id,     // id,
        0x0,    // target 0 rate (DDC/I2C)
        rate,   // target 1 rate (UART1)
        0x0,    // target 2 rate (UART2)
        0x0,    // target 3 rate (USB)
        0x0,    // target 4 rate (SPI)
        0x0,    // target 5 rate (reserved)
    };
    sendCmd(sizeof(cmdBuf), cmdBuf);
}
 
void rx_handler()
{
    buf[in++] = gps.getc();
    in &= (MAX_LENGTH-1);
    led1 = !led1;
}
 
int main()
{
    pc.baud(38400);//pc.baud(115200);
    gps.baud(38400);
 
    gps.attach( &rx_handler );

    //initiallize LEDs
    led1 = 1;
    led2 = 0;
    led3 = 0;
 
    // Configure GPS
    uint8_t cmdbuf[40];
    for (int i=0; i < 38; i++)
        cmdbuf[i] = 0;
    cmdbuf[0] = 0x06; // NAV-CFG5
    cmdbuf[1] = 0x24; // NAV-CFG5
    cmdbuf[2] = 0x00; // X2 bitmask
    cmdbuf[3] = 0x01; //    bitmask: dynamic model
    cmdbuf[4] = 0x04; // U1 automotive dyn model
    sendCmd(38, cmdbuf);
 
 
    // Modify these to control which messages are sent from module
    enableMsg(SOL_MSG, true);       // Enable soluton messages
 
    while (1)
    {
        if (in != out)
        {
            unsigned char cc = buf[out++];
            out &= (MAX_LENGTH-1);
 
            switch (state)
            {
                case 0:    // wait for sync 1 (0xB5)
                    ck1 = ck2 = 0;
                    checkOk = false;
                    if (cc == SYNC1)
                        state++;
                    break;
                case 1:    // wait for sync 2 (0x62)
                    if (cc == SYNC2)
                        state++;
                    else
                        state = 0;
                    break;
                case 2:    // wait for class code
                    code = cc;
                    ck1 += cc;
                    ck2 += ck1;
                    state++;
                    break;
                case 3:    // wait for Id
                    id = cc;
                    ck1 += cc;
                    ck2 += ck1;
                    state++;
                    break;
                case 4:    // wait for length uint8_t 1
                    length = cc;
                    ck1 += cc;
                    ck2 += ck1;
                    state++;
                    break;
                case 5:    // wait for length uint8_t 2
                    length |= (unsigned int) cc << 8;
                    ck1 += cc;
                    ck2 += ck1;
                    idx = 0;
                    state++;
                    if (length > MAX_LENGTH)
                        state= 0;
                    break;
                case 6:    // wait for <length> payload uint8_ts
                    data[idx++] = cc;
                    ck1 += cc;
                    ck2 += ck1;
                    if (idx >= length)
                    {
                        state++;
                    }
                    break;
                case 7:    // wait for checksum 1
                    chk1 = cc;
                    state++;
                    break;
                case 8: {  // wait for checksum 2
                    chk2 = cc;
                    checkOk = ck1 == chk1  &&  ck2 == chk2;
                    if (!checkOk)
                    {
                        led2 = 0;//led2 off when get a invalid message from GPS
                    }
                    else//get a valid message from GPS
                    {
                        led2 = 1;//led2 on when get a valid message from GPS

                        switch (code)
                        {
                            case 0x01:      // NAV-
                                switch (id)
                                {
                                    case SOL_MSG:  // NAV-SOL // we need this, ecef_pos_x, ecef_pos_y, ecef_pos_z, GPS_FIX
                                        led3 = (UINT_8(10) == 0x03);
                                        pc.printf("GPS_FIX = %x\r\n", UINT_8(10) );
                                        pc.printf("ECEF_POS_X = %x\r\n", INT_32(12) );//in cm
                                        pc.printf("ECEF_POS_Y = %x\r\n", INT_32(16) );//in cm
                                        pc.printf("ECEF_POS_Z = %x\r\n\r\n", INT_32(20) );//in cm
                                        break;
                                    default:
                                        break;
                                }
                                break;

                            default:
                                break;
                        }
                    }

                    state = 0;
                    break;
                }

                default:
                    break;
            }
        }
    }
}
 
void sendCmd (unsigned char len, uint8_t data[])
{
    unsigned char chk1 = 0, chk2 = 0;
 
    gps.putc(SYNC1);
    gps.putc(SYNC2);
    for (unsigned char ii = 0; ii < len; ii++) {
        gps.putc(data[ii]);
        chk1 += data[ii];
        chk2 += chk1;
    }
    gps.putc(chk1);
    gps.putc(chk2);
}
