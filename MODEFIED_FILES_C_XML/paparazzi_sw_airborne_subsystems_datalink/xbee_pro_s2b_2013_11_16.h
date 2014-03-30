/* FOR Xbee pro S2B module */

#ifndef XBEE_PRO_S2B_H
#define XBEE_PRO_S2B_H

#define XBEE_TX_ID 0x10
#define XBEE_RX_ID 0x90
#define XBEE_RFDATA_OFFSET 12

/******************** Modefied by YI and Edward ********************/
#define XBeeTransportPutTXHeader(_dev)  { \
  XBeeTransportPutUint8(_dev, XBEE_TX_ID); \
  XBeeTransportPutUint8(_dev, NO_FRAME_ID); \
  XBeeTransportPutUint8(_dev, (GROUND_STATION_ADDR >> 56) & 0xff); \
  XBeeTransportPutUint8(_dev, (GROUND_STATION_ADDR >> 48) & 0xff); \
  XBeeTransportPutUint8(_dev, (GROUND_STATION_ADDR >> 40) & 0xff); \
  XBeeTransportPutUint8(_dev, (GROUND_STATION_ADDR >> 32) & 0xff); \
  XBeeTransportPutUint8(_dev, (GROUND_STATION_ADDR >> 24) & 0xff); \
  XBeeTransportPutUint8(_dev, (GROUND_STATION_ADDR >> 16) & 0xff); \
  XBeeTransportPutUint8(_dev, (GROUND_STATION_ADDR >> 8) & 0xff); \
  XBeeTransportPutUint8(_dev, GROUND_STATION_ADDR & 0xff); \
  XBeeTransportPutUint8(_dev, 0xff); \
  XBeeTransportPutUint8(_dev, 0xfe); \
  XBeeTransportPutUint8(_dev, 0x00); \
  XBeeTransportPutUint8(_dev, TX_OPTIONS); \
}
/********************** End of modification ***********************/

/* 13 = frame_id + addr==8 + 3 + options */
#define XBeeTransportSizeOf(_dev, _x) XBeeAPISizeOf(_dev, _x+13)

#define XbeeGetRSSI(_payload) {}

#endif // XBEE_PRO_S2B_H
