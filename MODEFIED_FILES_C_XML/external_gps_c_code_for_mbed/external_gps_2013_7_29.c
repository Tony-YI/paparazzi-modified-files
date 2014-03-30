/**
 * uBlox UBX Protocol Reader - Wayne Holder
 * Ported to mbed - Michael Shimniok
 *
 * Modefied by YI and Edward
 *
 *PPRZ-message: ABCxxxxxxxDE
 *  A PPRZ_STX (0x99)
 *  B LENGTH (A->E)
 *  C PPRZ_DATA
 *    0 SENDER_ID
 *    1 MSG_ID
 *    2 MSG_PAYLOAD
 *    . DATA (messages.xml)
 *  D PPRZ_CHECKSUM_A (sum[B->C])
 *  E PPRZ_CHECKSUM_B (sum[ck_a])
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
Serial Xbee(p28,p27);//Serial for Xbee, which send message to the quadcopter.
DigitalOut led1(LED1);//LED for rx
DigitalOut led2(LED2);//LED on when get a valid message from GPS
DigitalOut led3(LED3);//LED on when get a 3D-Fix
DigitalOut led4(LED4);//LED for Xbee rx;
 
unsigned char buf[MAX_LENGTH];
int in=0;
int out=0;

/*----------------PPRZ DEFINATION-------------------*/
#define STX  0x99
#define AC_ID 1
#define MSG_ID 14 //message.xml class = datalink.
#define MSG_RECV_TARGET_ID 14
#define MSG_UART_ERROR 208
uint8_t pprz_ck1,pprz_ck2;
struct GpsState 
{
    uint8_t ac_id;
    uint8_t GPS_FIX;
    uint32_t ecef_pos_cm_x;
    uint32_t ecef_pos_cm_y;
    uint32_t ecef_pos_cm_z;
}__attribute__ ((packed));//this is very important

struct GpsState GPS;
void SendPprzXbeeMsg();//CALL THIS FUNCTION if you think it's right time to send the GPS data to quadcopter

uint8_t pprz_buf[MAX_LENGTH];
uint8_t pprz_data[MAX_LENGTH];
int pprz_in = 0;
int pprz_out = 0;
unsigned char pprz_state = 0;
unsigned char pprz_recv_ck1 = 0 ,pprz_recv_ck2 = 0,pprz_check_ok;
unsigned char recv_msg_id = 0;
uint8_t pprz_length = 0,pprz_index = 0;
void rx_Pprz_Xbee_Msg();
void parse_Pprz_Xbee_Msg();
/*----------------PPRZ DEFINATION END-------------------*/

void parse_GPS_Msg();
void enableMsg(unsigned char id, bool enable, int rate=1)
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
    pc.baud(9600);//pc.baud(115200);
    gps.baud(38400);
    Xbee.baud(57600);//Xbee baud rate as 57600.
    
    memset(&GPS,0,sizeof(struct GpsState));
    GPS.ac_id = AC_ID;
    memset(pprz_buf,0,MAX_LENGTH);
    memset(pprz_data,0,MAX_LENGTH);
    
    gps.attach( &rx_handler );
    Xbee.attach( &rx_Pprz_Xbee_Msg);

    //initiallize LEDs
    led1 = 1;
    led2 = 0;
    led3 = 0;
    led4 = 1;
    
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
        parse_GPS_Msg();
        parse_Pprz_Xbee_Msg();
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

/*---------------PPRZ IMPLEMENTATION-----------------*/
#define Put1byte(data){\
    Xbee.putc(data);\
}

#define PutUint8(data){\
    pprz_ck1+=data;\
    pprz_ck2+=pprz_ck1;\
    Put1byte(data);\
}

#define Put2byteByAddr(data){\
    PutUint8(*(const uint8_t *) data);\
    PutUint8(*(((const uint8_t *) data) + 1));\
}

#define Put4byteByAddr(data){\
    Put2byteByAddr(((const uint16_t *) data));\
    Put2byteByAddr((((const uint16_t *) data) + 1));\
}

#define PutStartMessage() {\
    Put1byte(STX);\
    Put1byte(sizeof(struct GpsState) + 4 + 2);\
    pprz_ck1 = sizeof(struct GpsState) + 4 + 2;\
    pprz_ck2 = sizeof(struct GpsState) + 4 + 2;\
    PutUint8(AC_ID);\
    PutUint8(MSG_ID);\
}

#define PutTrailer(){Put1byte(pprz_ck1);Put1byte(pprz_ck2);}

void SendPprzXbeeMsg()
{   
    PutStartMessage();
    //Message content Start//
    PutUint8(GPS.ac_id);
    PutUint8(GPS.GPS_FIX);
    Put4byteByAddr(&GPS.ecef_pos_cm_x);
    Put4byteByAddr(&GPS.ecef_pos_cm_y);
    Put4byteByAddr(&GPS.ecef_pos_cm_z);
    //Message content End//
    PutTrailer();
    return ;
}

void rx_Pprz_Xbee_Msg()
{
    pprz_buf[pprz_in++] = Xbee.getc();
    pprz_in &= (MAX_LENGTH-1);
    led4 = !led4;
}

void parse_Pprz_Xbee_Msg()
{
    if(pprz_in != pprz_out)
    {
        uint8_t temp = pprz_buf[pprz_out++];
        pprz_out &= (MAX_LENGTH - 1);
        
        switch(pprz_state)
        {
            case 0: //check for PPRZ_STX
            {
                //pc.printf("case 0, temp = %02x\r\n", temp);
                
                pprz_recv_ck1 = pprz_recv_ck2 = 0;
                pprz_check_ok = false;
                if(temp == STX)
                    pprz_state++;
                break;
            }
            case 1: //Check for length, should be smaller than MAX_LENGTH;
            {
                //pc.printf("case 1, temp = %02x\r\n", temp);
                
                pprz_length = temp;
                pprz_recv_ck1 += temp;
                pprz_recv_ck2 += pprz_recv_ck1;
                if(pprz_length > MAX_LENGTH)
                {
                    pc.printf("Error:Pprz Message length doesn't match %d\r\n",pprz_length);
                    pprz_state = 0;
                }
                else
                    pprz_state++;
                    
                break;
            }
            case 2://Check for AC_ID.
            {
                //pc.printf("case 2, temp = %02x\r\n", temp);
                
                if(temp != AC_ID)
                {
                    pc.printf("Error:AC_ID doesn't match %x\r\n",temp);
                    pprz_state = 0;
                }
                else
                {
                    pprz_recv_ck1 += temp;
                    pprz_recv_ck2 += pprz_recv_ck1;
                    pprz_state++;
                }
                break;
            }
            case 3://Check for Message ID.
            {
                //pc.printf("case 3, temp = %02x\r\n", temp);
                
                if(temp != MSG_RECV_TARGET_ID && temp != MSG_UART_ERROR)
                {
                    pc.printf("Error:MSG_ID doesn't match %x\r\n",temp);
                    pprz_state = 0;
                }
                else
                {
                    recv_msg_id = temp;
                    pprz_recv_ck1 += temp;
                    pprz_recv_ck2 += pprz_recv_ck1;
                    pprz_index = 0;
                    //pc.printf("MSG_LENGTH %d\r\n",pprz_length);
                    pprz_state++;
                }
                break;
            }
            case 4://Receving payload and computing check sum;
            {
                if(pprz_index < pprz_length - 6)
                {
                       // pc.printf("case 4, temp = %02x\r\n", temp);
                    pprz_data[pprz_index++] = temp;
                    pprz_recv_ck1 += temp;
                    pprz_recv_ck2 += pprz_recv_ck1;
                }
                else
                {
                    pprz_out--;//When checking pprz_index < pprz_length -6, pprz_out adds one more time so it needs to be rectracted.
                    pprz_state++;
                }
                break;
            }
            case 5://Receving check sum1
            {
                //pc.printf("case 5, temp = %02x\r\n", temp);
                   
                if(temp != pprz_recv_ck1)
                {
                    //pc.printf("!!!!!!!\r\n|");
                    //for(int count = 0;count < pprz_length;count++)
                    //{
                    //    pc.printf("%02x ",pprz_buf[count]);
                    //}
                    //pc.printf("|\r\n");
                    pprz_state = 0;
                    pc.printf("Error:Check sum 1 incorrect: expected %x, computed %x\r\n", temp, pprz_recv_ck1);
                }
                else
                    pprz_state++;
                break;
            }
            case 6://Receving check sum2
            {
                if(temp != pprz_recv_ck2)
                {
                    pprz_state = 0;
                    pc.printf("Error:Check sum 2 incorrec: expected %x,computed %x\r\n", temp, pprz_recv_ck2);
                }
                else
                    pprz_state++;
                break;
            }
            case 7://Printting out Message Content;
            {
                switch(recv_msg_id)
                {
                    case MSG_RECV_TARGET_ID:
                    {
                        pc.printf("RECV_MSG:AC_ID %x\r\n",*(uint8_t *) &pprz_data[0]);
                        pc.printf("RECV_MSG:GPS_FIX %x\r\n",*(uint8_t *) &pprz_data[1]);
                        pc.printf("RECV_MSG:ECEF_POS_X %d\r\n",*(int32_t *) &pprz_data[2]);
                        pc.printf("RECV_MSG:ECEF_POS_Y %d\r\n",*(int32_t *) &pprz_data[6]);
                        pc.printf("RECV_MSG:ECEF_POS_Z %d\r\n\r\n",*(int32_t *) &pprz_data[10]);
                        break;
                    }
                    case MSG_UART_ERROR:
                    {
                        pc.printf("UART_ERROR:\r\n");
                        pc.printf("UART_ERROR_FRAME_CNT:%d\r\n",*(uint16_t *) &pprz_data[4]);
                        break;
                    }
                    default:
                    {
                        pc.printf("Unkonw MSG\r\n");
                    }
                }
                pprz_state = 0;
                break;
            }
            default:
            {
                pc.printf("Error:Entering Default, please check the State Machine\r\n");
                break;
            }
        } 
    }
}
/*-----------------------------------------*/


void parse_GPS_Msg()
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
                                        led3 = (UINT_8(10) == 0x03);//get a GPS 3D-FIX
                                        pc.printf("GPS_FIX = %x\r\n", UINT_8(10) );
                                        pc.printf("ECEF_POS_X = %d\r\n", INT_32(12) );//in cm
                                        pc.printf("ECEF_POS_Y = %d\r\n", INT_32(16) );//in cm
                                        pc.printf("ECEF_POS_Z = %d\r\n\r\n", INT_32(20) );//in cm
                                        GPS.GPS_FIX = UINT_8(10);
                                        GPS.ecef_pos_cm_x = INT_32(12);
                                        GPS.ecef_pos_cm_y = INT_32(16);
                                        GPS.ecef_pos_cm_z = INT_32(20);
                                        SendPprzXbeeMsg();
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
