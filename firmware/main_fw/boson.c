#include "boson.h"

#include <stdio.h>
#include <generated/csr.h>

#include "boson/EnumTypes.h"
#include "logger.h"
#include "timer.h"

volatile static uint16_t boson_crc;

volatile static uint32_t _seq = 0;

#define UINT32_LE(a) (a << 24) & 0xFF, (a << 16) & 0xFF, (a << 8) & 0xFF, a & 0xFF

#define UART_EV_TX 0x1
#define UART_EV_RX 0x2

/* Prototypes */
static uint8_t boson_uart_read(void);
static int boson_uart_read_nonblock(void);
static void boson_uart_write(uint8_t c);
static void boson_uart_write_escaped(uint8_t c);
static void boson_uart_init(void);
static void boson_uart_sync(void);
static void boson_uart_write_array(const uint8_t *array, uint32_t len);

static uint8_t boson_uart_read(void) {
    uint8_t c;
    while (boson_boson_uart_rxempty_read()) {
    }

    c = boson_boson_uart_rxtx_read();
    boson_boson_uart_ev_pending_write(UART_EV_RX);
    return c;
}

static int boson_uart_read_nonblock(void) {
    return (boson_boson_uart_rxempty_read() == 0);
}

static void boson_uart_write(uint8_t c) {
    boson_crc = ByteCRC16((int)c, boson_crc);
    while (boson_boson_uart_txfull_read()) {
    }

    boson_boson_uart_rxtx_write(c);
    boson_boson_uart_ev_pending_write(UART_EV_TX);
}

static void boson_uart_write_escaped(uint8_t c) {
    if ((c == START_FRAME_BYTE) || (c == END_FRAME_BYTE) || (c == ESCAPE_BYTE)) {
        boson_uart_write(ESCAPE_BYTE);

        switch (c) {
            case END_FRAME_BYTE:
                c = ESCAPED_END_FRAME_BYTE;
                break;
            case START_FRAME_BYTE:
                c = ESCAPED_START_FRAME_BYTE;
                break;
            case ESCAPE_BYTE:
                c = ESCAPED_ESCAPE_BYTE;
                break;
            default:
                break;
        }
    }

    boson_uart_write(c);
}

static void boson_uart_init(void) {
    boson_boson_uart_ev_pending_write(boson_boson_uart_ev_pending_read());
    boson_boson_uart_ev_enable_write(UART_EV_TX | UART_EV_RX);
}

static void boson_uart_sync(void) {
    while (boson_boson_uart_txfull_read()) {
    }
}

static void boson_uart_write_array(const uint8_t *array, uint32_t len) {
    for (int i = 0; i < len; i++) {
        boson_uart_write_escaped(array[i]);
    }
}

/* Similar to the FLIR SDK functions, but without buffering */
FLR_RESULT dispatcher_tx(uint32_t seqNum, FLR_FUNCTION fnID, const uint8_t *sendData, const uint32_t sendBytes) {
    uint8_t tmp_array[4];

    boson_uart_write(START_FRAME_BYTE);

    /* Reset CRC, START_FRAME_BYTE not included in calculation */
    boson_crc = FLIR_CRC_INITIAL_VALUE;
    boson_uart_write(0); /* Channel ID: Command Channel */

    /* Send Sequence number */
    UINT_32ToByte(seqNum, (const uint8_t *)tmp_array);
    boson_uart_write_array(tmp_array, 4);

    /* Send function ID */
    UINT_32ToByte((const uint32_t)fnID, (const uint8_t *)tmp_array);
    boson_uart_write_array(tmp_array, 4);

    /* Send 0xFFFFFFFF */
    UINT_32ToByte(0xFFFFFFFF, (const uint8_t *)tmp_array);
    boson_uart_write_array(tmp_array, 4);

    /* Send sendData */
    if (sendBytes > 0) {
        boson_uart_write_array(sendData, sendBytes);
    }

    /* Send out the CRC */
    uint8_t crcbyte0 = ((boson_crc >> 8) & 0xFF);
    uint8_t crcbyte1 = (boson_crc & 0xFF);
    boson_uart_write_escaped(crcbyte0);
    boson_uart_write_escaped(crcbyte1);

    boson_uart_write(END_FRAME_BYTE);

    return R_SUCCESS;
}

FLR_RESULT dispatcher_rx(uint8_t *recvData, uint32_t *recvBytes) {
    /* Setup a timeout interval */
    int timeout = 250;
    int timeout_count = 0;

    // char str[256];
    // char* str_p = str;

    FLR_RESULT errorCode = R_UART_RECEIVE_TIMEOUT;

    uint32_t max_len = *recvBytes;
    *recvBytes = 0;

    uint32_t len = 0;

    //str_p += snprintf(str_p, sizeof(str) - (str_p - str),"bsn << ");
    while ((++timeout_count < timeout) && (errorCode != R_SUCCESS)) {
        while (boson_uart_read_nonblock()) {
            recvData[len] = boson_uart_read();

            /* Skip forward looking for START_OF_FRAME */
            if (len == 0) {
                if (recvData[0] != START_FRAME_BYTE)
                    continue;
            }

            //str_p += snprintf(str_p, sizeof(str) - (str_p - str),"%02x ", recvData[len]);

            if (len > 2) {
                if (recvData[len] == END_FRAME_BYTE) {
                    if (recvData[len - 1] != ESCAPED_END_FRAME_BYTE) {
                        /* End of frame */
                        errorCode = R_SUCCESS;
                        *recvBytes = len + 1;
                        break;
                    }
                }
            }

            /* Basic bounds protection */
            if (len < max_len) {
                len++;
            } else {
                errorCode = R_SDK_PKG_BUFFER_OVERFLOW;
                break;
            }
        }

        busy_wait(1);
    }

    //str_p += snprintf(str_p, sizeof(str) - (str_p - str),"(%u ms)", timeout_count);
    //log_printf("Boson: Dispatcher: %s", str);

    return errorCode;
}

FLR_RESULT dispatcher(FLR_FUNCTION fnID, const uint8_t *sendData, const uint32_t sendBytes) {
    FLR_RESULT r = R_SYM_UNSPECIFIED_FAILURE;
    uint8_t recvData[64];
    uint32_t recvBytes = (sizeof(recvData) / sizeof(uint8_t));
    uint32_t seq = _seq++;

    log_printf("Boson: dispatcher: fnID=%s", FLR_FUNCTION_c_str(fnID));

    r = dispatcher_tx(seq, fnID, sendData, sendBytes);
    if (r != R_SUCCESS) {
        return r;
    }

    boson_uart_sync();

    /* Listen to the RX bytes, to check error code. */
    int retry = 3;
    while (retry--) {
        r = dispatcher_rx(recvData, &recvBytes);
        if (r == R_SUCCESS) {
            /* Check CRC */
            if (recvBytes > 4) {
                uint16_t calcCRC = calcFlirCRC16Bytes(recvBytes - 2, recvData + 1);
                if (calcCRC != 0) {
                    return R_UART_UNSPECIFIED_FAILURE;
                }
            }
            /* Check seq number */
            uint32_t rx_seq = 0;
            byteToUINT_32(recvData + 2, &rx_seq);
            if (seq != rx_seq) {
                return R_UART_UNSPECIFIED_FAILURE;
            }

            /* Check result code */
            uint32_t ret_code;
            byteToUINT_32(recvData + 10, &ret_code);
            r = ret_code;

            return R_SUCCESS;
        }

        log_printf("Boson: Dispatcher: %s ret=%s (%u)", FLR_FUNCTION_c_str(fnID), FLR_RESULT_R_c_str(r), r);
    }

    return r;
}

void boson_init(void) {
    boson_boson_reset_out_write(0);
    busy_wait(500);
    boson_boson_reset_out_write(1);
    boson_uart_init();
    busy_wait(4500);
    boson_uart_write(0);
    boson_uart_write(0);

    /* Ping the camera a few times, takes ~3s after bootup before it will respond */
    FLR_RESULT r = R_UART_RECEIVE_TIMEOUT;
    for (int tries = 20; tries > 0 && r != R_SUCCESS; --tries) {
        r = dispatcher(BOSON_GETCAMERASN, 0, 0);
    }

    if (r == R_SUCCESS) {
        /* Basic setup of boson core */
        // dispatcher(TESTRAMP_SETTYPE, (const uint8_t[]){0, UINT32_LE(FLR_TESTRAMP_INCREMENTING)}, 5 );
        // dispatcher(GAO_SETTESTRAMPSTATE, (const uint8_t[]){UINT32_LE(FLR_ENABLE)}, 4);

        dispatcher(TELEMETRY_SETSTATE, (const uint8_t[]){UINT32_LE(FLR_DISABLE)}, 4);
        dispatcher(TELEMETRY_SETLOCATION, (const uint8_t[]){UINT32_LE(FLR_TELEMETRY_LOC_TOP)}, 4);

        dispatcher(DVO_SETDISPLAYMODE, (const uint8_t[]){UINT32_LE(FLR_DVO_CONTINUOUS)}, 4); /* FLR_DVO_CONTINUOUS */
        dispatcher(DVO_SETANALOGVIDEOSTATE, (const uint8_t[]){UINT32_LE(FLR_DISABLE)}, 4);   /* Analog Off */
        dispatcher(DVO_SETOUTPUTFORMAT, (const uint8_t[]){UINT32_LE(FLR_DVO_IR16)}, 4);      /* Mode: IR16 */
        dispatcher(DVO_SETTYPE, (const uint8_t[]){UINT32_LE(FLR_DVO_TYPE_MONO16)}, 4);       /* Type: MONO16 */
        dispatcher(DVO_SETVIDEOSTANDARD, (const uint8_t[]){UINT32_LE(FLR_DVO_NTSC)}, 4);     /* Video Standard NTSC, 60/30Hz */
        dispatcher(DVO_APPLYCUSTOMSETTINGS, 0, 0);
        //dispatcher(COLORLUT_SETID, (const uint8_t[]){UINT32_LE(_settings.pallete)}, 4); /* Colour LUT: Ironbow */
        busy_wait(10);
        dispatcher(COLORLUT_SETCONTROL, (const uint8_t[]){UINT32_LE(FLR_DISABLE)}, 4);
        busy_wait(10);
        dispatcher(GAO_SETAVERAGERSTATE, (const uint8_t[]){UINT32_LE(FLR_ENABLE)}, 4);

        busy_wait(500);
        dispatcher(BOSON_RUNFFC, 0, 0);
    }
}

void boson_set_lut(uint32_t lut) {
    dispatcher(COLORLUT_SETID, (const uint8_t[]){0x00, 0x00, 0x00, lut}, 4); /* Colour LUT: Ironbow */
}

void boson_set_averager(uint32_t en) {
    dispatcher(GAO_SETAVERAGERSTATE, (const uint8_t[]){UINT32_LE(en)}, 4);
}
