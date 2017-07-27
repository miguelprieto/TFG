#ifndef CONFIG_H
#define CONFIG_H

#define CONFIG_XBEE_PATH "/dev/ttyUSB0"
#define CONFIG_XBEEADDRESS_GRANDE XBeeAddress(0x00, 0x01)
#define CONFIG_XBEEADDRESS_PICCOLO XBeeAddress(0x00, 0x02)

// Servizio XBeeTap (cattura e trasmissione pacchetti zigbee arbitrari)
#define CONFIG_XBEETAP_PORT 6668

// Servizio XBeeRtsp (programmazione wireless)
#define CONFIG_XBEERTSP_GRANDE_PORT 6661
#define CONFIG_XBEERTSP_PICCOLO_PORT 6662

// Servizio XBeeStdio (console robot)
#define CONFIG_STDIO_GRANDE_BASEPORT 6670
#define CONFIG_STDIO_GRANDE_ENDPOINTS { 1, 2, 7 }
#define CONFIG_STDIO_PICCOLO_BASEPORT 6680
#define CONFIG_STDIO_PICCOLO_ENDPOINTS { 1, 2 }

#endif // CONFIG_H
