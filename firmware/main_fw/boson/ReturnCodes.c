
#include "ReturnCodes.h"

char* FLR_RESULT_R_c_str(FLR_RESULT r) {
    switch (r) {
        case R_SUCCESS:
            return "R_SUCCESS";
        case R_UART_UNSPECIFIED_FAILURE:
            return "R_UART_UNSPECIFIED_FAILURE";
        case R_UART_PORT_FAILURE:
            return "R_UART_PORT_FAILURE";
        case R_UART_RECEIVE_TIMEOUT:
            return "R_UART_RECEIVE_TIMEOUT";
        case R_UART_PORT_ALREADY_OPEN:
            return "R_UART_PORT_ALREADY_OPEN";
        case R_SDK_API_UNSPECIFIED_FAILURE:
            return "R_SDK_API_UNSPECIFIED_FAILURE";
        case R_SDK_API_NOT_DEFINED:
            return "R_SDK_API_NOT_DEFINED";
        case R_SDK_PKG_UNSPECIFIED_FAILURE:
            return "R_SDK_PKG_UNSPECIFIED_FAILURE";
        case R_SDK_PKG_BUFFER_OVERFLOW:
            return "R_SDK_PKG_BUFFER_OVERFLOW";
        case R_SDK_DSPCH_UNSPECIFIED_FAILURE:
            return "R_SDK_DSPCH_UNSPECIFIED_FAILURE";
        case R_SDK_DSPCH_SEQUENCE_MISMATCH:
            return "R_SDK_DSPCH_SEQUENCE_MISMATCH";
        case R_SDK_DSPCH_ID_MISMATCH:
            return "R_SDK_DSPCH_ID_MISMATCH";
        case R_SDK_DSPCH_MALFORMED_STATUS:
            return "R_SDK_DSPCH_MALFORMED_STATUS";
        case R_SDK_TX_UNSPECIFIED_FAILURE:
            return "R_SDK_TX_UNSPECIFIED_FAILURE";
        case R_CAM_RX_UNSPECIFIED_FAILURE:
            return "R_CAM_RX_UNSPECIFIED_FAILURE";
        case R_CAM_DSPCH_UNSPECIFIED_FAILURE:
            return "R_CAM_DSPCH_UNSPECIFIED_FAILURE";
        case R_CAM_DSPCH_BAD_CMD_ID:
            return "R_CAM_DSPCH_BAD_CMD_ID";
        case R_CAM_DSPCH_BAD_PAYLOAD_STATUS:
            return "R_CAM_DSPCH_BAD_PAYLOAD_STATUS";
        case R_CAM_PKG_UNSPECIFIED_FAILURE:
            return "R_CAM_PKG_UNSPECIFIED_FAILURE";
        case R_CAM_PKG_INSUFFICIENT_BYTES:
            return "R_CAM_PKG_INSUFFICIENT_BYTES";
        case R_CAM_PKG_EXCESS_BYTES:
            return "R_CAM_PKG_EXCESS_BYTES";
        case R_CAM_PKG_BUFFER_OVERFLOW:
            return "R_CAM_PKG_BUFFER_OVERFLOW";
        case R_CAM_API_UNSPECIFIED_FAILURE:
            return "R_CAM_API_UNSPECIFIED_FAILURE";
        case R_CAM_API_INVALID_INPUT:
            return "R_CAM_API_INVALID_INPUT";
        case R_CAM_TX_UNSPECIFIED_FAILURE:
            return "R_CAM_TX_UNSPECIFIED_FAILURE";
        case R_API_RX_UNSPECIFIED_FAILURE:
            return "R_API_RX_UNSPECIFIED_FAILURE";
        case R_CAM_FEATURE_NOT_ENABLED:
            return "R_CAM_FEATURE_NOT_ENABLED";
        default:
            return "FLR_RESULT Unknown";
    }
}

char* FLR_RESULT_c_str(FLR_RESULT r) {
    switch (r) {
        case FLR_OK:
            return "FLR_OK";
        case FLR_ERROR:
            return "FLR_ERROR";
        case FLR_NOT_READY:
            return "FLR_NOT_READY";
        case FLR_RANGE_ERROR:
            return "FLR_RANGE_ERROR";
        case FLR_CHECKSUM_ERROR:
            return "FLR_CHECKSUM_ERROR";
        case FLR_BAD_ARG_POINTER_ERROR:
            return "FLR_BAD_ARG_POINTER_ERROR";
        case FLR_DATA_SIZE_ERROR:
            return "FLR_DATA_SIZE_ERROR";
        case FLR_UNDEFINED_FUNCTION_ERROR:
            return "FLR_UNDEFINED_FUNCTION_ERROR";
        case FLR_ILLEGAL_ADDRESS_ERROR:
            return "FLR_ILLEGAL_ADDRESS_ERROR";
        case FLR_BAD_OUT_TYPE:
            return "FLR_BAD_OUT_TYPE";
        case FLR_BAD_OUT_INTERFACE:
            return "FLR_BAD_OUT_INTERFACE";
        case FLR_COMM_PORT_NOT_OPEN:
            return "FLR_COMM_PORT_NOT_OPEN";
        case FLR_COMM_INVALID_PORT_ERROR:
            return "FLR_COMM_INVALID_PORT_ERROR";
        case FLR_COMM_RANGE_ERROR:
            return "FLR_COMM_RANGE_ERROR";
        case FLR_ERROR_CREATING_COMM:
            return "FLR_ERROR_CREATING_COMM";
        case FLR_ERROR_STARTING_COMM:
            return "FLR_ERROR_STARTING_COMM";
        case FLR_ERROR_CLOSING_COMM:
            return "FLR_ERROR_CLOSING_COMM";
        case FLR_COMM_CHECKSUM_ERROR:
            return "FLR_COMM_CHECKSUM_ERROR";
        case FLR_COMM_NO_DEV:
            return "FLR_COMM_NO_DEV";
        case FLR_COMM_TIMEOUT_ERROR:
            return "FLR_COMM_TIMEOUT_ERROR";
        //case FLR_COMM_ERROR_WRITING_COMM:
        //    return "FLR_COMM_ERROR_WRITING_COMM";
        case FLR_COMM_ERROR_READING_COMM:
            return "FLR_COMM_ERROR_READING_COMM";
        case FLR_COMM_COUNT_ERROR:
            return "FLR_COMM_COUNT_ERROR";
        case FLR_OPERATION_CANCELED:
            return "FLR_OPERATION_CANCELED";
        case FLR_UNDEFINED_ERROR_CODE:
            return "FLR_UNDEFINED_ERROR_CODE";
        case FLR_LEN_NOT_SUBBLOCK_BOUNDARY:
            return "FLR_LEN_NOT_SUBBLOCK_BOUNDARY";
        case FLR_CONFIG_ERROR:
            return "FLR_CONFIG_ERROR";
        case FLR_I2C_ERROR:
            return "FLR_I2C_ERROR";
        case FLR_CAM_BUSY:
            return "FLR_CAM_BUSY";
        case FLR_HEATER_ERROR:
            return "FLR_HEATER_ERROR";
        case FLR_WINDOW_ERROR:
            return "FLR_WINDOW_ERROR";
        case FLR_VBATT_ERROR:
            return "FLR_VBATT_ERROR";
        case R_SYM_UNSPECIFIED_FAILURE:
            return "R_SYM_UNSPECIFIED_FAILURE";
        case R_SYM_INVALID_POSITION_ERROR:
            return "R_SYM_INVALID_POSITION_ERROR";
        case FLR_RES_NOT_AVAILABLE:
            return "FLR_RES_NOT_AVAILABLE";
        case FLR_RES_NOT_IMPLEMENTED:
            return "FLR_RES_NOT_IMPLEMENTED";
        case FLR_RES_RANGE_ERROR:
            return "FLR_RES_RANGE_ERROR";
        case FLR_SYSTEMINIT_XX_ERROR:
            return "FLR_SYSTEMINIT_XX_ERROR";
        case FLR_SDIO_XX_ERROR:
            return "FLR_SDIO_XX_ERROR";
        case FLR_STOR_SD_XX_ERROR:
            return "FLR_STOR_SD_XX_ERROR";
        case FLR_USB_VIDEO_XX_ERROR:
            return "FLR_USB_VIDEO_XX_ERROR";
        case FLR_USB_CDC_XX_ERROR:
            return "FLR_USB_CDC_XX_ERROR";
        case FLR_USB_MSD_XX_ERROR:
            return "FLR_USB_MSD_XX_ERROR";
        case FLR_NET_XX_ERROR:
            return "FLR_NET_XX_ERROR";
        case FLR_BT_XX_ERROR:
            return "FLR_BT_XX_ERROR";
        case FLR_FLASH_XX_ERROR:
            return "FLR_FLASH_XX_ERROR";

        default:
            return "FLR_RESULT Unknown";
    }
}
