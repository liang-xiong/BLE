/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "ble_dfu_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"
#include "nrf_timer.h"
#include "nrf_drv_timer.h"
#include "nrf_dfu_req_handler.h"
#include "nrf_bootloader_info.h"

#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define HARDWARE_NUMBER                 "HW_1_0"
#define SOFTWARE_NUMBER                 "SW_16_0_0"
#define FIRMWARE_NUMBER                 "L_FW_1_0_9"

#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE        512                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        512                                     /**< UART RX buffer size. */
#define BLE_MAX_DATA            512

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */

#define UART_RX_TIMEOUT_INTERVAL        3                                           /**< uart rx timeout interval(ms). */

//要连接设备的名称，在扫描的时候搜索此名称
static char const  m_target_periph_name[]="QJHKASH";
static char set_target_periph_name[32]={0};
static char check_target_periph_name[64] = {0};

const nrf_drv_timer_t TIMER_UART_RX = NRF_DRV_TIMER_INSTANCE(2);                    /**< Timer ID: Timer2. */
static uint8_t  UART_RX_BUF[UART_RX_BUF_SIZE] = {0};                                /**< uart receive buffer. */
static uint16_t UART_RX_STA = 0;

BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
BLE_DFU_C_DEF(m_ble_dfu_c);
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

//hpy
static uint8_t mac_addr[BLE_GAP_ADDR_LEN];
static uint8_t set_m_addr[BLE_GAP_ADDR_LEN];
//用于连接buttonbless的dfu服务
/**@brief NUS UUID. */
static ble_uuid_t const m_dfu_uuid =
{
    .uuid = BLE_UUID_DEVICE_INFORMATION_SERVICE,
    .type = BLE_UUID_TYPE_BLE
};

//use it to connect periph nus service
//static ble_uuid_t const m_nus_uuid =
//{
//	.uuid = BLE_UUID_NUS_SERVICE,
//	.type = NUS_SERVICE_UUID_TYPE
//};


static uint8_t m_target_periph_addr[6] = {0};

typedef enum
{
    BLE_STATUS_DISCONNECTED = 0,// ble disconnected
    BLE_STATUS_CONNECTED        // ble connected
} ble_status_t;

static ble_status_t ble_status = BLE_STATUS_DISCONNECTED;

typedef enum
{
    BLE_DFU_NOTFOUND = 0,// ble disconnected
    BLE_DFU_FOUND        // ble connected
} ble_dfu_server_t;

static ble_dfu_server_t ble_dfu_server = BLE_DFU_NOTFOUND;


#define B_ADDR_LEN    6

/*********************************************************************
 * @fn      Util_convertBdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @param   pAddr - BD address
 *
 * @return  BD address as a string
 */
char *Util_convertBdAddr2Str(uint8_t *pAddr)
{
  uint8_t     charCnt;
  char        hex[] = "0123456789ABCDEF";
  static char str[(2*B_ADDR_LEN)+3];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for (charCnt = B_ADDR_LEN; charCnt > 0; charCnt--)
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  pStr = NULL;

  return str;
}

/*********************************************************************
 * @fn      Util_convertHex2Str
 *
 * @brief   将HEX转成String，用于转化广播数据和扫描回调数据
 *
 * @param   Hex - Hex
 *          len - Hex len
 *
 * @return  Hex as a string
 */
char *Util_convertHex2Str(uint8_t *Hex, uint16_t Len)
{
  uint8_t     charCnt;
  char        hex[] = "0123456789ABCDEF";
  static char str[(2*31)+3];    // 广播数据最大31字节
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  for (charCnt = Len; charCnt > 0; charCnt--)
  {
    *pStr++ = hex[*Hex >> 4];
    *pStr++ = hex[*Hex++ & 0x0F];
  }
  pStr = NULL;

  str[2*Len+2] = '\0';
  return str;
}

void Util_convertStr2Hex(unsigned char *pbDest, unsigned char *pbSrc, int nLen)
{
    char h1,h2;
    unsigned char s1,s2;
    int i;

    for (i=0; i<nLen; i++)
    {
        h1 = pbSrc[2*i];
        h2 = pbSrc[2*i+1];

        s1 = toupper(h1) - 0x30;
        if (s1 > 9)
            s1 -= 7;

        s2 = toupper(h2) - 0x30;
        if (s2 > 9)
            s2 -= 7;

        pbDest[i] = s1*16 + s2;
    }
}

/**
 * @brief @ref NRF_DFU_OP_PROTOCOL_VERSION response details.
 */
typedef struct
{
    uint8_t version; //!< Protocol version.
} ble_dfu_response_protocol_t;

/**
 * @brief @ref NRF_DFU_OP_HARDWARE_VERSION response details.
 */
typedef struct
{
    uint32_t part;    //!< Hardware part, from FICR register.
    uint32_t variant; //!< Hardware variant, from FICR register.
    struct
    {
        uint32_t rom_size;      //!< ROM size, in bytes.
        uint32_t ram_size;      //!< RAM size, in bytes.
        uint32_t rom_page_size; //!< ROM flash page size, in bytes.
    } memory;
} ble_dfu_response_hardware_t;

/**
 * @brief @ref NRF_DFU_OP_FIRMWARE_VERSION response details.
 */
typedef struct
{
    nrf_dfu_firmware_type_t type; //!< Firmware type.
    uint32_t version;             //!< Firmware version.
    uint32_t addr;                //!< Firmware address in flash.
    uint32_t len;                 //!< Firmware length in bytes.
} ble_dfu_response_firmware_t;

/**
 * @brief @ref NRF_DFU_OP_OBJECT_SELECT response details.
 */
typedef struct
{
    uint32_t max_size; //!< Maximum size of selected object.
    uint32_t offset;   //!< Current offset.
    uint32_t crc;      //!< Current CRC.
} ble_dfu_response_select_t;

/**
 * @brief @ref NRF_DFU_OP_OBJECT_CREATE response details.
 */
typedef struct
{
    //TODO not used in Bootloader
    //uint32_t offset;                    //!< Current offset
    //uint32_t crc;                       //!< Current CRC.
} ble_dfu_response_create_t;

/**
 * @brief @ref NRF_DFU_OP_OBJECT_WRITE response details.
 */
typedef struct
{
    uint32_t offset; //!< Used only when packet receipt notification is used.
    uint32_t crc;    //!< Used only when packet receipt notification is used.
} ble_dfu_response_write_t;

/**
 * @brief @ref NRF_DFU_OP_CRC_GET response details.
 */
typedef struct
{
    uint32_t offset; //!< Current offset.
    uint32_t crc;    //!< Current CRC.
} ble_dfu_response_crc_t;

/**
 * @brief @ref NRF_DFU_OP_PING response details.
 */
typedef struct
{
    uint8_t id; //!< The received ID which is echoed back.
} ble_dfu_response_ping_t;

/**
 * @brief @ref NRF_DFU_OP_MTU_GET response details.
 */
typedef struct
{
    uint16_t size; //!< The MTU size as specified by the local transport.
} ble_dfu_response_mtu_t;

/**
 * @brief DFU response message.
 */
typedef struct
{
    nrf_dfu_op_t request;    //!< Requested operation.
    nrf_dfu_result_t result; //!< Result of the operation.
    union {
        ble_dfu_response_protocol_t protocol; //!< Protocol version response.
        ble_dfu_response_hardware_t hardware; //!< Hardware version response.
        ble_dfu_response_firmware_t firmware; //!< Firmware version response.
        ble_dfu_response_select_t select;     //!< Select object response..
        ble_dfu_response_create_t create;     //!< Create object response..
        ble_dfu_response_write_t write;       //!< Write object response.
        ble_dfu_response_crc_t crc;           //!< CRC response.
        ble_dfu_response_ping_t ping;         //!< Ping response.
        ble_dfu_response_mtu_t mtu;           //!< MTU response.
        uint8_t ext_err;
    };
} __attribute__((packed)) ble_dfu_response_t;

/**
 * @brief @ref NRF_DFU_OP_FIRMWARE_VERSION request details.
 */
typedef struct
{
    uint8_t image_number; //!< Index of the firmware.
} ble_dfu_request_firmware_t;

/**
 * @brief @ref NRF_DFU_OP_OBJECT_SELECT request details.
 */
typedef struct
{
    uint8_t object_type; //!< Object type. See @ref nrf_dfu_obj_type_t.
} ble_dfu_request_select_t;

/**
 * @brief @ref NRF_DFU_OP_OBJECT_CREATE request details.
 */
typedef struct
{
    uint8_t object_type;  //!< Object type. See @ref nrf_dfu_obj_type_t.
    uint32_t object_size; //!< Object size in bytes.
} __attribute__((packed)) ble_dfu_request_create_t;

/**
 * @brief @ref NRF_DFU_OP_OBJECT_WRITE request details.
 */
typedef struct
{
    //TODO not usable
    //uint8_t  const * p_data; //!< Data.
    //uint16_t         len;    //!< Length of data in @ref nrf_dfu_request_write_t::p_data.
} ble_dfu_request_write_t;

/**
 * @brief @ref NRF_DFU_OP_PING request details.
 */
typedef struct
{
    uint8_t id; //!< Ping ID that will be returned in response.
} ble_dfu_request_ping_t;

/**
 * @brief @ref NRF_DFU_OP_MTU_GET request details.
 */
typedef struct
{
    //TODO not used in Bootloader
    //uint16_t size;          //!< Transport MTU size in bytes.
} ble_dfu_request_mtu_t;

/**
 * @brief @ref NRF_DFU_OP_RECEIPT_NOTIF_SET request details.
 */
typedef struct
{
    uint16_t target; //!< Target PRN.
} ble_dfu_request_prn_t;

/**
 *@brief DFU request.
 */
typedef struct
{
    nrf_dfu_op_t request; //!< Requested operation.
    union {
        ble_dfu_request_firmware_t firmware; //!< Firmware version request.
        ble_dfu_request_select_t select;     //!< Select object request.
        ble_dfu_request_create_t create;     //!< Create object request.
        ble_dfu_request_write_t write;       //!< Write object request.
        ble_dfu_request_ping_t ping;         //!< Ping.
        ble_dfu_request_mtu_t mtu;           //!< MTU size request.
        ble_dfu_request_prn_t prn;           //!< Set receipt notification request.
    };
} __attribute__((packed)) ble_dfu_request_t;
//name change flag
bool set_name_flag = false;

static bool cccd_change_flag = false;
//为了不从app竟然boot，直接连接bootloader进行调试避免不打印log问题
// static bool cccd_change_flag = true;
typedef ret_code_t(* reset_filter_mac_t)(uint8_t * periph_addr);
reset_filter_mac_t reset_filter_mac = NULL;
ret_code_t filter_settings_change(uint8_t * periph_addr)
{
    ret_code_t err_code;

    cccd_change_flag = true;

    err_code = nrf_ble_scan_all_filter_remove(&m_scan);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_ADDR_FILTER, periph_addr);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_ADDR_FILTER, false);
    APP_ERROR_CHECK(err_code);

    return err_code;
}

ret_code_t filter_settings_reset()
{
    ret_code_t err_code;

    cccd_change_flag = false;
	set_name_flag =false;

    err_code = nrf_ble_scan_all_filter_remove(&m_scan);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_dfu_uuid);
    APP_ERROR_CHECK(err_code);

    //err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    // APP_ERROR_CHECK(err_code);
			
    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    APP_ERROR_CHECK(err_code);
	err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, set_target_periph_name);
    APP_ERROR_CHECK(err_code);

	err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);
    return err_code;
}

//用户读取通知消息的定义的函数指针
typedef void(*ble_dfu_write_rsp_t)(ble_dfu_c_t * p_ble_dfu_c, ble_dfu_c_evt_t const * p_ble_dfu_evt);

ble_dfu_write_rsp_t ble_dfu_write_rsp = NULL;





//用户读取通知消息的定义的函数指针
typedef void(*ble_dfu_hvx_notify_t)(ble_dfu_c_t * p_ble_dfu_c, ble_dfu_c_evt_t const * p_ble_dfu_evt);

ble_dfu_hvx_notify_t ble_dfu_hvx_notify = NULL;

void ble_dfu_enterboot_confirm(ble_dfu_c_t * p_ble_dfu_c, ble_dfu_c_evt_t const * p_ble_dfu_evt)
{
    ret_code_t err_code;

    if (   p_ble_dfu_evt->p_data[0] == DFU_OP_RESPONSE_CODE
        && p_ble_dfu_evt->p_data[1] == DFU_OP_ENTER_BOOTLOADER
        && p_ble_dfu_evt->p_data[2] == DFU_RSP_SUCCESS)
    {
        //向GATT服务器发送一个句柄值确认
        sd_ble_gattc_hv_confirm(p_ble_dfu_c->conn_handle, p_ble_dfu_c->handles.dfu_write_handle);
        // ble_dfu_c_send_indicate(&p_ble_dfu_c, NULL, 0);
        //连接mac地址加一
        for (uint8_t i = 0; i < BLE_GAP_ADDR_LEN; i++)
        {
            m_target_periph_addr[i] = mac_addr[i];
        }
        m_target_periph_addr[0]++;

        //连接DfuTarg
        NRF_LOG_INFO("Connecting to DfuTarg %02x%02x%02x%02x%02x%02x",
              mac_addr[0],
              mac_addr[1],
              mac_addr[2],
              mac_addr[3],
              mac_addr[4],
              mac_addr[5]);

        reset_filter_mac = filter_settings_change;

        //调用断连程序断开连接
        err_code = sd_ble_gap_disconnect(p_ble_dfu_c->conn_handle,
                                     BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }

        if(NRF_SUCCESS != err_code)
        {
            printf("AT+BLEDFU:ERR\r\nerr_code:%ld\r\n", err_code);
        }
        else
        {
            printf("AT+BLEDFU:OK\r\n");
        }
    }
}

void ble_dfu_response(ble_dfu_c_t * p_ble_dfu_c, ble_dfu_c_evt_t const * p_ble_dfu_evt)
{
    NRF_LOG_INFO("data_len:%d", p_ble_dfu_evt->data_len);

    if (p_ble_dfu_evt->p_data[0] != NRF_DFU_OP_RESPONSE)
    {
        
        printf("NO_RES_ERR:%d\r\n", p_ble_dfu_evt->p_data[0]);
    }

    ble_dfu_response_t* resp = (ble_dfu_response_t*)(p_ble_dfu_evt->p_data + 1);

    if (resp->result != NRF_DFU_RES_CODE_SUCCESS) {
        NRF_LOG_INFO("Response Error %x", resp->result);
        printf("ERR:%d\r\n", resp->result);
    }
    else
    {
        printf("OK\r\n");
    }
}

void ble_dfu_receive_notify_response(ble_dfu_c_t * p_ble_dfu_c, ble_dfu_c_evt_t const * p_ble_dfu_evt)
{
    NRF_LOG_INFO("data_len:%d", p_ble_dfu_evt->data_len);

    if (p_ble_dfu_evt->p_data[0] != NRF_DFU_OP_RESPONSE)
    {
        printf("NO_RES_ERR:%d\r\n", p_ble_dfu_evt->p_data[0]);
    }

    ble_dfu_response_t* resp = (ble_dfu_response_t*)(p_ble_dfu_evt->p_data + 1);

    if (resp->result != NRF_DFU_RES_CODE_SUCCESS) {
        NRF_LOG_INFO("Response Error %x", resp->result);
        printf("ERR:%d\r\n", resp->result);
    }
    else
    {
        printf("OK\r\n");
    }
}

void ble_dfu_receive_cversion_response(ble_dfu_c_t * p_ble_dfu_c, ble_dfu_c_evt_t const * p_ble_dfu_evt)
{
    NRF_LOG_INFO("data_len:%d", p_ble_dfu_evt->data_len);

    if (p_ble_dfu_evt->p_data[0] != NRF_DFU_OP_RESPONSE &&
       p_ble_dfu_evt->p_data[1] != DFU_OP_GET_VERSION)
    {
        printf("NO_RES_ERR:%d\r\n", p_ble_dfu_evt->p_data[0]);
    }
    else
    {
        NRF_LOG_INFO("ble_dfu_receive_cversion_response");
        sd_ble_gattc_hv_confirm(p_ble_dfu_c->conn_handle, p_ble_dfu_c->handles.dfu_write_handle);
        uint8_t version_str[256] = {0};
        memcpy(version_str ,(uint8_t *)p_ble_dfu_evt->p_data+2, p_ble_dfu_evt->data_len - 2);
        printf("OK\r\nCVER:%s\r\n", version_str);

    }
}


void ble_dfu_crc_response(ble_dfu_c_t * p_ble_dfu_c, ble_dfu_c_evt_t const * p_ble_dfu_evt)
{
    NRF_LOG_INFO("data_len:%d", p_ble_dfu_evt->data_len);

    if (p_ble_dfu_evt->p_data[0] != NRF_DFU_OP_RESPONSE)
    {
        printf("NO_RES_ERR:%d\r\n", p_ble_dfu_evt->p_data[0]);
    }

    ble_dfu_response_t* resp = (ble_dfu_response_t*)(p_ble_dfu_evt->p_data + 1);

    if (resp->result != NRF_DFU_RES_CODE_SUCCESS) {
        NRF_LOG_INFO("Response Error %x", resp->result);
        printf("ERR:%d\r\n", resp->result);
    }
    else
    {
        printf("OK\r\nCRC:%lu,OFFSET:%lu\r\n",resp->crc.crc, resp->crc.offset);
    }
}

uint32_t swapInt32(uint32_t value)
{
     return ((value & 0x000000FF) << 24) |
               ((value & 0x0000FF00) << 8) |
               ((value & 0x00FF0000) >> 8) |
               ((value & 0xFF000000) >> 24) ;
}

void ble_dfu_slelect_response(ble_dfu_c_t * p_ble_dfu_c, ble_dfu_c_evt_t const * p_ble_dfu_evt)
{
    if (p_ble_dfu_evt->p_data[0] != NRF_DFU_OP_RESPONSE)
    {
        printf("NO_RES_ERR:%d\r\n", p_ble_dfu_evt->p_data[0]);
    }

    ble_dfu_response_t* resp = (ble_dfu_response_t*)(p_ble_dfu_evt->p_data + 1);

    if (resp->result != NRF_DFU_RES_CODE_SUCCESS) {
        NRF_LOG_INFO("Response Error %x", resp->result);
        printf("ERR:%d\r\n", resp->result);
    }
    else
    {
        printf("OK\r\nMAX_SIZE:%lu,CRC:%lu,OFFSET:%lu\r\n", resp->select.max_size, resp->select.crc, resp->select.offset);
    }
}

ret_code_t send_enter_bootloader(void)
{
    ret_code_t err_code;
    uint8_t p_data = 1;
    err_code = ble_dfu_c_write(&m_ble_dfu_c, &p_data, 1);
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
    {
        NRF_LOG_ERROR("Failed sending message. Error 0x%x. ", err_code);
        return err_code;
    }
    return err_code;
}



ret_code_t ble_dfu_write_ctl(uint8_t * req, uint16_t data_len)
{
    ret_code_t err_code;

    err_code = ble_dfu_c_write(&m_ble_dfu_c, req, data_len);
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
    {
        NRF_LOG_ERROR("Failed sending message. Error 0x%x. ", err_code);
        return err_code;
    }
    return err_code;
}

ret_code_t ble_dfu_write_data(uint8_t * req, uint16_t data_len)
{
    ret_code_t err_code;

    err_code = ble_dfu_c_write_cmd(&m_ble_dfu_c, req, data_len);
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
    {
        NRF_LOG_ERROR("Failed sending message. Error 0x%x. ", err_code);
        return err_code;
    }
    return err_code;
}


static size_t request_size(ble_dfu_request_t* req)
{
    switch (req->request) {
    case NRF_DFU_OP_OBJECT_CREATE:
        return 1 + sizeof(req->create);
    case NRF_DFU_OP_RECEIPT_NOTIF_SET:
        return 1 + sizeof(req->prn);
    case NRF_DFU_OP_OBJECT_SELECT:
        return 1 + sizeof(req->select);
    case NRF_DFU_OP_MTU_GET:
        return 1; // NOT sizeof(req->mtu);
    case NRF_DFU_OP_OBJECT_WRITE:
        return 1 + sizeof(req->write);
    case NRF_DFU_OP_PING:
        return 1 + sizeof(req->ping);
    case NRF_DFU_OP_FIRMWARE_VERSION:
        return 1 + sizeof(req->firmware);
    case NRF_DFU_OP_PROTOCOL_VERSION:
    case NRF_DFU_OP_CRC_GET:
    case NRF_DFU_OP_OBJECT_EXECUTE:
    case NRF_DFU_OP_HARDWARE_VERSION:
    case NRF_DFU_OP_ABORT:
    case NRF_DFU_OP_RESPONSE:
    case NRF_DFU_OP_INVALID:
        return 1;
    }
    return 0;
}

static bool ble_send_request(ble_dfu_request_t* req)
{
    size_t size = request_size(req);
    NRF_LOG_INFO("ble_send_request:%d", size);
    if (size == 0) {
        NRF_LOG_INFO("Unknown size");
        return false;
    }

    return ble_dfu_write_ctl((uint8_t*)req, size);

    return true;
}

static bool ble_dfu_object_select(uint8_t type, uint32_t* offset, uint32_t* crc)
{
    NRF_LOG_INFO("Select object %d: ", type);
    ble_dfu_request_t req = {
        .request = NRF_DFU_OP_OBJECT_SELECT,
        .select.object_type = type,
    };

    bool b = ble_send_request(&req);
    if (!b) {
        return false;
    }

    return true;
}


static bool ble_dfu_object_create(uint8_t type, uint32_t size)
{
    NRF_LOG_INFO("Create object %d: ", type);

    //传输的对象数据需要注意大小端匹配
    //传输为小端传输，正好芯片默认为小端
    //所以不用转换
    ble_dfu_request_t req = {
        .request = NRF_DFU_OP_OBJECT_CREATE,
        .create.object_type = type,
        .create.object_size = size,
    };

    bool b = ble_send_request(&req);
    if (!b) {
        return false;
    }

    return true;
}

static bool ble_dfu_object_execute(void)
{
    NRF_LOG_INFO("Object Execute: ");
    ble_dfu_request_t req = {
        .request = NRF_DFU_OP_OBJECT_EXECUTE,
    };

    bool b = ble_send_request(&req);
    if (!b) {
        return false;
    }
    return true;
}

static bool ble_dfu_get_crc(void)
{
    NRF_LOG_INFO("Get CRC: ");
    ble_dfu_request_t req = {
        .request = NRF_DFU_OP_CRC_GET,
    };

    bool b = ble_send_request(&req);
    if (!b) {
        return false;
    }

    return true;
}

static bool ble_dfu_receive_notify(uint8_t prn)
{
    NRF_LOG_INFO("Set packet receive notification: ");
    ble_dfu_request_t req = {
        .request = NRF_DFU_OP_RECEIPT_NOTIF_SET,
        .prn.target = prn,
    };

    bool b = ble_send_request(&req);
    if (!b) {
        return false;
    }

    return true;
}


uint32_t dfu_bootloader_start(void)
{
    uint32_t err_code;

    NRF_LOG_DEBUG("In dfu_bootloader_start\r\n");
    NRF_LOG_FLUSH();

    err_code = sd_power_gpregret_clr(0, 0xffffffff);
    VERIFY_SUCCESS(err_code);

    err_code = sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
    VERIFY_SUCCESS(err_code);

    // Signal that DFU mode is to be enter to the power management module
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_DFU);

    return NRF_SUCCESS;
}

/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error)
{
    NRF_LOG_INFO("nus_error_handler");
    APP_ERROR_HANDLER(nrf_error);
}

static void dfu_error_handler(uint32_t nrf_error)
{
    NRF_LOG_INFO("dfu_error_handler");
    APP_ERROR_HANDLER(nrf_error);
}
/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}



/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
	{

        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
          err_code = p_scan_evt->params.connecting_err.err_code;
          APP_ERROR_CHECK(err_code);
        } break;

        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
          ble_gap_evt_connected_t const * p_connected =
                           p_scan_evt->params.connected.p_connected;

         // for (uint8_t i = 0; i < BLE_GAP_ADDR_LEN; i++)
         // {
         //     mac_addr[i] = p_connected->peer_addr.addr[i];
         // }

         // Scan is automatically stopped by the connection.
         NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                  p_connected->peer_addr.addr[0],
                  p_connected->peer_addr.addr[1],
                  p_connected->peer_addr.addr[2],
                  p_connected->peer_addr.addr[3],
                  p_connected->peer_addr.addr[4],
                  p_connected->peer_addr.addr[5]
                  );
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
         NRF_LOG_INFO("Scan timed out.");
         scan_start();
        } break;

        default:
         break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;
	int8_t txpwr_level = 0;
    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);
	
	//set tx_pwr
	err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT,BLE_CONN_HANDLE_INVALID,txpwr_level);
    APP_ERROR_CHECK(err_code);
    // nrf_ble_scan_filter_set(&m_scan, SCAN_ADDR_FILTER, &m_dfu_uuid);
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_dfu_uuid);
    APP_ERROR_CHECK(err_code);

    //err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    //APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    APP_ERROR_CHECK(err_code);
	// name filter
	err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
    APP_ERROR_CHECK(err_code);

	err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);
    // // 配置扫描的参数
    // init_scan.p_scan_param = &m_scan_params;

    // // 配置连接的参数
    // init_scan.p_conn_param = &m_conn_params;
    
    // // 初始化扫描
    // err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    // APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
    ble_dfu_c_on_db_disc_evt(&m_ble_dfu_c, p_evt);
}


/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
/*
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    NRF_LOG_DEBG("Receiving data.");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {
            ret_val = app_uart_put(p_data[i]);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
    if (p_data[data_len-1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }
    if (ECHOBACK_BLE_UART_DATA)
    {
        // Send data back to the peripheral.
        do
        {
            ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
}
*/


/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
	//uint32_t ret_val = -10;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            nrf_drv_timer_disable(&TIMER_UART_RX);
            
            UNUSED_VARIABLE(app_uart_get(&UART_RX_BUF[UART_RX_STA]));
            UART_RX_STA++;  // Record the uart received data frame length
			
            nrf_drv_timer_enable(&TIMER_UART_RX);
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

bool nus_flag = false;
uint16_t test_msg_size = 0;
uint8_t  p_string[] ="wewins";
/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */
/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{

    ret_code_t err_code;
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Nordic UART Service.");
            break;

		case BLE_NUS_C_EVT_NUS_TX_EVT:
            //ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
			if (nus_flag == true)
			{
				if(*(p_ble_nus_evt->p_data) != '\n')
				{
	 				//printf("RCV,MSG:%s,LEN=%d\r\n",p_ble_nus_evt->p_data,p_ble_nus_evt->data_len);
					if(0 == strncmp((char *)p_ble_nus_evt->p_data,(char *)p_string,p_ble_nus_evt->data_len))
						printf("AT+SENDMSG?\r\n+SENDMSG:send and rcv ok\r\nOK\r\n");
					else
				
						printf("AT+SENDMSG?\r\n+SENDMSG:send and rcv fail\r\nOK\r\n");
				}
				nus_flag = false;
			}
			memset(p_ble_nus_evt->p_data,0,m_ble_nus_max_data_len);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */

static void ble_dfu_c_evt_handler(ble_dfu_c_t * p_ble_dfu_c, ble_dfu_c_evt_t const * p_ble_dfu_evt)
{
    ret_code_t err_code;

    switch (p_ble_dfu_evt->evt_type)
    {
        case BLE_DFU_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Dfu discovery complete.");
            err_code = ble_dfu_c_handles_assign(p_ble_dfu_c, p_ble_dfu_evt->conn_handle, &p_ble_dfu_evt->handles);
            NRF_LOG_INFO("BLE_DFU_C_EVT_DISCOVERY_COMPLETE err_code:%d", err_code);
            APP_ERROR_CHECK(err_code);

            if ( !cccd_change_flag )
            {
                err_code = ble_dfu_c_write_notif_enable(p_ble_dfu_c);
            }
            else
            {
                err_code = ble_dfu_c_write_test_enable(p_ble_dfu_c);
            }
            NRF_LOG_INFO("BLE_DFU_C_EVT_DISCOVERY_COMPLETE err_code2:%d", err_code);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Nordic DFU Service.");
            ble_dfu_server = BLE_DFU_FOUND;

            break;

        case BLE_DFU_C_EVT_DFU_HVX_EVT:

            NRF_LOG_INFO("BLE_DFU_C_EVT_DFU_HVX_EVT");
            if (NULL != ble_dfu_hvx_notify)
            {
                //这是一个函数指针指向的函数
                 ble_dfu_hvx_notify(p_ble_dfu_c, p_ble_dfu_evt);
            }
            ble_dfu_hvx_notify = NULL;

            break;
        case BLE_DFU_C_EVT_DFU_WRITE_RSP_EVT:

            NRF_LOG_INFO("BLE_DFU_C_EVT_DFU_WRITE_RSP_EVT");
            if (NULL != ble_dfu_write_rsp)
            {
                //这是一个函数指针指向的函数
                 ble_dfu_write_rsp(p_ble_dfu_c, p_ble_dfu_evt);
            }
            ble_dfu_write_rsp = NULL;

            break;

        case BLE_DFU_C_EVT_DISCONNECTED:
            ble_dfu_server = BLE_DFU_NOTFOUND;
            NRF_LOG_INFO("Dfu disconnected.");
            
            scan_start();
            break;
    }
}


/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);


int8_t rssi_value = 0;
ble_gap_evt_rssi_changed_t rssi_changed={0};
/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
	ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    ble_gap_evt_connected_t const * p_connected_evt = &p_gap_evt->params.connected;
    int num = p_ble_evt->header.evt_id;

    switch (num)
    {

        case BLE_GAP_EVT_CONNECTED:
            ble_status = BLE_STATUS_CONNECTED;

            NRF_LOG_INFO("Connected ble_evt_handler");
            err_code = ble_dfu_c_handles_assign(&m_ble_dfu_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
			err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
			APP_ERROR_CHECK(err_code);
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
			
			
            for (uint8_t i = 0; i < BLE_GAP_ADDR_LEN; i++)
            {
                mac_addr[i] = p_connected_evt->peer_addr.addr[i];
            }
			uint16_t m_conn_handle = p_gap_evt->conn_handle;
			// rssi start
			sd_ble_gap_rssi_start(m_conn_handle,0,0);
            NRF_LOG_INFO("\r\nConnected. conn_DevAddr: %s\nConnected. conn_handle:0x%04x\nConnected. conn_Param: %d,%d,%d,%d\r\n",
            Util_convertBdAddr2Str((uint8_t*)p_connected_evt->peer_addr.addr),
            p_gap_evt->conn_handle,
            p_connected_evt->conn_params.min_conn_interval,
            p_connected_evt->conn_params.max_conn_interval,
            p_connected_evt->conn_params.slave_latency,
            p_connected_evt->conn_params.conn_sup_timeout
            );
            break;

		case BLE_GAP_EVT_RSSI_CHANGED:
			if(rssi_value != p_gap_evt->params.rssi_changed.rssi)
			{
				rssi_value = p_gap_evt->params.rssi_changed.rssi;
	 			//printf("\r\nrssi= %ddbm\r\n",rssi_value);
			}
			break;

			
        case BLE_GAP_EVT_DISCONNECTED:
            ble_status = BLE_STATUS_DISCONNECTED;
	        NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
	                    p_gap_evt->conn_handle,
						 p_gap_evt->params.disconnected.reason);
			if (NULL != reset_filter_mac)
			{
	 			reset_filter_mac(m_target_periph_addr);
				reset_filter_mac = NULL;
			}
			else
			{


				filter_settings_reset();
			}
		//	if(set_name_flag)
		//		filter_settings_reset();

            
            // if (cccd_change_flag)
            // {
            //     filter_settings_reset();
            // }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_ble_dfu_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}
/**@brief Function for initializing the Nordic UART Service (NUS) client. */
// static void nus_c_init(void)
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler   = ble_nus_c_evt_handler;
    init.error_handler = nus_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}
static void dfu_c_init(void)
{
    ret_code_t       err_code;
    ble_dfu_c_init_t init;

    init.evt_handler   = ble_dfu_c_evt_handler;
    init.error_handler = dfu_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_dfu_c_init(&m_ble_dfu_c, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for check uart received data whether is AT cmd or Pass-through data.
 *
 * @param[in] pBuffer   pointer of string.
 * @param[in] length    Length of the data.
 *
 * @retval 1 AT command
 * @retval 0 Pass-through data
 */
static uint8_t AT_cmd_check_valid(uint8_t *pBuffer, uint16_t length)
{
    // check whether is AT cmd or not   
    if(length < 2) {
        return 0;
    }

    if((strncmp((char*)pBuffer, "AT", 2)!=0) && (NULL != strstr((char*)pBuffer, "\r\n"))) {
        return 0;
    }

    return 1;
}
/**@brief Function for process AT command.
 *
 * @param[in] pBuffer   pointer of string.
 * @param[in] length    Length of the data.
 */
static void AT_cmd_handle(uint8_t *pBuffer, uint16_t length)
{
    ret_code_t err_code;
    // *(pBuffer+length) = '\0';
    ble_gap_addr_t p_addr;
    NRF_LOG_INFO("length:%d", length);
    NRF_LOG_INFO("strlen:%d", strlen("AT+MAC?\r\n"))
    NRF_LOG_INFO("%s", pBuffer);
    // length--;
    
    //测试通讯
    // AT test: AT?\r\n
    if((length == strlen("AT?\r\n")) && (strncmp((char*)pBuffer, "AT?", strlen("AT?")) == 0))
    {
        printf("AT:OK\r\n"); //初始化进入enter_bootloader_handle回调函数
    }
    //软件复位
    // System soft reset: AT+RESET\r\n
    else if((length == strlen("AT+RESET\r\n")) && (strncmp((char*)pBuffer, "AT+RESET", strlen("AT+RESET")) == 0))
    {
        NVIC_SystemReset(); // Restart the system by default    
    }
    // 获取本机mac地址: AT+MAC?\r\n
    else if((length == strlen("AT+MAC?\r\n")) && (strncmp((char*)pBuffer, "AT+MAC?", strlen("AT+MAC?")) == 0))
    {
        NRF_LOG_INFO("hpy mac");
        // Get BLE address.
        err_code = sd_ble_gap_addr_get(&p_addr);
        APP_ERROR_CHECK(err_code);

        printf("OK\r\nAT+MAC:%s\r\n", Util_convertBdAddr2Str(p_addr.addr));
    }
//修改本机蓝牙的地址 AT+MAC=0x123456789ABC
	else if ((length == strlen("AT+MAC=0x123456789ABC\r\n"))&&(strncmp((char *)pBuffer,"AT+MAC=",strlen("AT+MAC="))==0))
	{

		if(ble_status == BLE_STATUS_CONNECTED)
		{
			memset(p_addr.addr,0,6);
			unsigned char buf[20] ={0};
			unsigned char hex[20] ={0};
			sscanf((char *)pBuffer,"AT+MAC=0x%srn",buf);
			
			Util_convertStr2Hex(hex, buf, strlen((char *)buf));
			p_addr.addr[0] = hex[5];
			p_addr.addr[1] = hex[4];
			p_addr.addr[2] = hex[3];
			p_addr.addr[3] = hex[2];
			p_addr.addr[4] = hex[1];
			p_addr.addr[5] = hex[0];
			p_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
			err_code = sd_ble_gap_addr_set(&p_addr);
			if(err_code == NRF_SUCCESS)
			{
				printf("SET ADDR OK\r\n");
			}
			else
				printf("SET ADDR FAIL\r\n");
		}
		else
			printf("ERR:NOT CONNECT\r\n");
	}

    //获取本机版本
    // Hardware/firmware/software version check: AT+VER?\r\n
    else if((length == strlen("AT+VER?\r\n")) && (strncmp((char*)pBuffer, "AT+VER?", strlen("AT+VER?")) == 0))
    {
        printf("OK\r\nVER:%s\r\n", FIRMWARE_NUMBER);
    }

    else if((length == strlen("AT+VERSION?\r\n")) && (strncmp((char*)pBuffer, "AT+VERSION?", strlen("AT+VERSION?")) == 0))
    {
        printf("AT+VERSION?\r\n+VERSION:%s\r\nOK\r\n", FIRMWARE_NUMBER);
	}

    //查询主机连接状态
    // BLE connection status check: AT+STATUS?\r\n
    else if((length == strlen("AT+STATUS?\r\n"))&&(strncmp((char*)pBuffer, "AT+STATUS?", strlen("AT+STATUS?")) == 0))
    {
        if(ble_status == BLE_STATUS_CONNECTED)
			printf("AT+STATUS?\r\n+STATUS:connect\r\nOK\r\n");
		else 
			printf("AT+STATUS?\r\n+STATUS:disconn\r\nOK\r\n");
    }

    //主机和从机断开连接
    // Disconnet connection: AT+DISCON\r\n
    else if((length == strlen("AT+DISCON\r\n")) && (strncmp((char*)pBuffer, "AT+DISCON", strlen("AT+DISCON")) == 0))
    {
        if(ble_status == BLE_STATUS_CONNECTED)
        {
            err_code = sd_ble_gap_disconnect(m_ble_dfu_c.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if(err_code == NRF_SUCCESS)
            {
                printf("AT+DISCON:OK\r\n");
            }
            else
            {
                printf("AT+DISCON:FAIL\r\n");
            }
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            printf("AT+DISCON:ERP\r\n");
        }
    }

    //获取从机的mac地址
    else if ((length == strlen("AT+CMAC?\r\n")) && (strncmp((char*)pBuffer, "AT+CMAC?", strlen("AT+CMAC?")) == 0))
    {
        if(ble_status == BLE_STATUS_CONNECTED)
        {
            printf("OK\r\nAT+CMAC:%s\r\n", Util_convertBdAddr2Str(mac_addr));
        }
        else
        {
            printf("AT+CMAC:FAIL\r\n");
        }
    }

    //向从机发送写控制指令指令AT+DFUWRITECTRL=%d,\"%s\"\r\n
    else if ((length > strlen("AT+DFUWRITECTRL=\r\n")) && (strncmp((char*)pBuffer, "AT+DFUWRITECTRL=", strlen("AT+DFUWRITECTRL=")) == 0))
    {
        if(BLE_DFU_FOUND == ble_dfu_server)
        {
            uint32_t data_len;
            uint8_t at_rw_data[BLE_MAX_DATA];

            sscanf((char*)pBuffer, "AT+DFUWRITECTRL=%ld,\"%s\"rn", &data_len, at_rw_data);
            //如果数据长度为奇数将其变为偶数
            if (data_len%2 == 0)
            {
                NRF_LOG_INFO("%s", at_rw_data);
                uint16_t length = data_len>2;
                Util_convertStr2Hex(at_rw_data, at_rw_data, length);
                ble_dfu_hvx_notify = ble_dfu_response;
                err_code = ble_dfu_write_ctl(at_rw_data, length);
            }
            else
            {
                printf("INPUT ERR\n");
            }
        }
        else
        {
            printf("AT+DFUWRITECTRL:ERR\r\nNOT FOUND DFU SERVER\r\n");
        }
    }

    //向从机发送读数据指令AT+DFUWRITEDATA=%d,\"%s\"\r\n
    else if ((length > strlen("AT+DFUWRITEDATA=\r\n")) && (strncmp((char*)pBuffer, "AT+DFUWRITEDATA=", strlen("AT+DFUWRITEDATA=")) == 0))
    {
        if(BLE_DFU_FOUND == ble_dfu_server)
        {
            uint32_t data_len;
            uint8_t at_rw_data[BLE_MAX_DATA] = {0};
            sscanf((char*)pBuffer, "AT+DFUWRITEDATA=%lu,\"%s\"rn", &data_len, (char *)at_rw_data);
            //如果数据长度为奇数将其变为偶数
            if (data_len%2 == 0)
            {
                uint16_t length = data_len/2;
                Util_convertStr2Hex(at_rw_data, at_rw_data, length);
                ble_dfu_hvx_notify = ble_dfu_response;
                err_code = ble_dfu_write_data(at_rw_data, length);

                if(err_code == NRF_SUCCESS)
                {
                    printf("OK\r\n");
                }
            }
            else
            {
                printf("ERR\n");
            }
        }
        else
        {
            printf("AT+DFUWRITEDATA:ERR\r\nNOT FOUND DFU SERVER\r\n");
        }
    }

    //查询mtu参数
    else if ((length == strlen("AT+OPMTUGET?\r\n")) && (strncmp((char*)pBuffer, "AT+OPMTUGET?", strlen("AT+OPMTUGET?")) == 0))
    {
         if(ble_status == BLE_STATUS_CONNECTED)
        {
          NRF_LOG_INFO("OPMTUGET");
            printf("OK\r\nAT+OPMTUGET:%d\r\n", nrf_ble_gatt_eff_mtu_get(&m_gatt, m_ble_dfu_c.conn_handle) - OPCODE_LENGTH - HANDLE_LENGTH);
        }
        else
        {
            printf("ERR:NOT CONNECT\r\n");
        }
    }


    //搜索从机DFU服务
    else if ((length == strlen("AT+BLEDFU\r\n")) && (strncmp((char*)pBuffer, "AT+BLEDFU", strlen("AT+BLEDFU")) == 0))
    {
        //初始化进入enter_bootloader_handle回调函数
        if (BLE_DFU_FOUND == ble_dfu_server)
        {
            ble_dfu_hvx_notify = ble_dfu_enterboot_confirm;
            err_code = send_enter_bootloader();
        }
        else
        {
            printf("AT+BLEDFU:ERR\r\nNOT FOUND DFU SERVER\r\n");
        }
    }

    //AT+CMAC=<十六进制地址>\r\n,例如：AT+CMAC=0xDE254C4B0447
    else if ((length == strlen("AT+CMAC=0x123456789ABC\r\n")) && (strncmp((char*)pBuffer, "AT+CMAC=", strlen("AT+CMAC=")) == 0))
    {
        uint8_t periph_addr[6];

        sscanf((char*)pBuffer, "AT+CMAC=0x%02hhx%02hhx%02hhx%02hhx%02hhx%02hhxrn",
				&periph_addr[5],
				&periph_addr[4],
				&periph_addr[3],
				&periph_addr[2],
				&periph_addr[1],
				&periph_addr[0]);
        err_code = filter_settings_change(periph_addr);
        if(err_code == NRF_SUCCESS)
        {
            printf("OK\r\n");
        }
        else
        {
            printf("ERR:%ld\r\n", err_code);
        }
    }

    //AT+OBJSELECT=<type>\r\n
    else if ((length > strlen("AT+OBJSELECT=\r\n")) && (strncmp((char*)pBuffer, "AT+OBJSELECT=", strlen("AT+OBJSELECT=")) == 0))
    {
        if(BLE_DFU_FOUND == ble_dfu_server)
        {
            uint32_t type, offset, crc;
            sscanf((char*)pBuffer, "AT+OBJSELECT=%ld\"rn", &type);
            ble_dfu_hvx_notify = ble_dfu_slelect_response;//添加一个钩子函数用来处理响应
            if (ble_dfu_object_select( type, &offset, &crc))
            {
                err_code = 1;
            }
            else
            {
                err_code = 0;
            }
        }
        else
        {
            printf("AT+OBJSELECT:ERR\r\nNOT FOUND DFU SERVER\r\n");
        }
    }

    //AT+OBJCREATE=<type>,<size>\r\n
    else if ((length > strlen("AT+OBJCREATE=\r\n")) && (strncmp((char*)pBuffer, "AT+OBJCREATE=", strlen("AT+OBJCREATE=")) == 0))
    {
        if(BLE_DFU_FOUND == ble_dfu_server)
        {
            uint32_t type, size;
            sscanf((char*)pBuffer, "AT+OBJCREATE=%lu,%lu\"rn", &type, &size);
            ble_dfu_hvx_notify = ble_dfu_response;//添加一个钩子函数用来处理响应
            if (ble_dfu_object_create( type, size))
            {
                err_code = 1;
            }
            else
            {
                err_code = 0;
            }
        }
        else
        {
            printf("AT+OBJCREATE:ERR\r\nNOT FOUND DFU SERVER\r\n");
        }
    }

    //AT+OBJEXECUTE\r\n
    else if ((length == strlen("AT+OBJEXECUTE\r\n")) && (strncmp((char*)pBuffer, "AT+OBJEXECUTE", strlen("AT+OBJEXECUTE")) == 0))
    {
        if(BLE_DFU_FOUND == ble_dfu_server)
        {
            ble_dfu_hvx_notify = ble_dfu_response;//添加一个钩子函数用来处理响应
            if (ble_dfu_object_execute())
            {
                NRF_LOG_INFO("AT+OBJEXECUTE ERR");
            }
            else
            {
                NRF_LOG_INFO("AT+OBJEXECUTE OK");
            }
        }
        else
        {
            printf("AT+OBJEXECUTE:ERR\r\nNOT FOUND DFU SERVER\r\n");
        }
    }

    //AT+OPCRCGET?\r\n
    else if ((length == strlen("AT+OPCRCGET?\r\n")) && (strncmp((char*)pBuffer, "AT+OPCRCGET?", strlen("AT+OPCRCGET?")) == 0))
    {
        if(BLE_DFU_FOUND == ble_dfu_server)
        {
            ble_dfu_hvx_notify = ble_dfu_crc_response;//添加一个钩子函数用来处理响应
            ble_dfu_get_crc();
        }
        else
        {
            printf("AT+OPCRCGET?:ERR\r\nNOT FOUND DFU SERVER\r\n");
        }
    }

    //AT+PING\r\n
    else if ((length > strlen("AT+RECIENOOTIF\r\n")) && (strncmp((char*)pBuffer, "AT+RECIENOOTIF=", strlen("AT+RECIENOOTIF=")) == 0))
    {
        NRF_LOG_INFO("RECIENOOTIF start");
        if(BLE_DFU_FOUND == ble_dfu_server)
        {
            uint8_t prn;
            sscanf((char*)pBuffer, "AT+RECIENOOTIF=%hhd\"rn", &prn);
            NRF_LOG_INFO("RECIENOOTIF prn:%d", prn);
            ble_dfu_hvx_notify = ble_dfu_receive_notify_response;//添加一个钩子函数用来处理响应
            ble_dfu_receive_notify(prn);
        }
        else
        {
            printf("AT+RECIENOOTIF:ERR\r\nNOT FOUND DFU SERVER\r\n");
        }
    }

    //本机进入bootloader模式
    else if ((length == strlen("AT+SERIALDFU\r\n")) && (strncmp((char*)pBuffer, "AT+SERIALDFU", strlen("AT+SERIALDFU")) == 0))
    {
        //串口数据没立刻法发出就已经进入bootloader
        //所以这条消息没有AT返回值，可以使用发送AT?\r\n来确认是否
        dfu_bootloader_start();
    }

        //向从机发送启动
    else if ((length == strlen("AT+CVER?\r\n")) && (strncmp((char*)pBuffer, "AT+CVER?", strlen("AT+CVER?")) == 0))
    {
        if(BLE_DFU_FOUND == ble_dfu_server)
        {
            ble_dfu_hvx_notify = ble_dfu_receive_cversion_response;//添加一个钩子函数用来处理响应
            ble_dfu_get_version_send(&m_ble_dfu_c);
        }
        else
        {
            printf("AT+CVER?:ERR\r\nNOT FOUND DFU SERVER\r\n");
        }
    }

    //断开蓝牙并停止刷新
    else if ((length == strlen("AT+CLOSEBLE\r\n")) && (strncmp((char*)pBuffer, "AT+CLOSEBLE", strlen("AT+CLOSEBLE")) == 0))
    {
    }

    //主机启动/停止扫描

    //获取从机的固件版本
    else if ((length == strlen("AT+CVER?\r\n")) && (strncmp((char*)pBuffer, "AT+CVER?", strlen("AT+CVER?")) == 0))
    {
    }

    //设置连接参数
    //设置射频发送功率
	else if((length>strlen("AT+TXPWR=\r\n"))&&(strncmp((char *)pBuffer,"AT+TXPWR=",strlen("AT+TXPWR="))==0))
	{

		int8_t txpwr_level;
		sscanf((char *)pBuffer,"AT+TXPWR=%hhdrn",&txpwr_level);
		//ble_gap_evt_adv_report_t TX_POWER;
		err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT,BLE_CONN_HANDLE_INVALID,txpwr_level);
		//printf("\r\nTXPWR=%d\r\n",TX_POWER.tx_power);
		if(err_code == NRF_SUCCESS)
		{
			printf("\r\ntxpwr_set OK\r\n");
		}
		else
			printf("\r\ntxpwr_set FAIL\r\n");
	}
//获取从蓝牙的rssi的值
	else if((length == strlen("AT+RSSI?\r\n"))&&(strncmp((char *)pBuffer,"AT+RSSI?",strlen("AT+RSSI?"))==0))
	{

		if(ble_status == BLE_STATUS_CONNECTED)
		{

			printf("AT+RSSI?\r\n+RSSI:%d\r\nOK\r\n",rssi_value);
		}
		else
			printf("AT+RSSI?\r\n+RSSI:fail\r\nOK\r\n");
	}

//向从蓝牙发送字符串
	else if(strncmp((char *)pBuffer,"AT+SENDMSG?",strlen("AT+SENDMSG?"))==0)
	{

		uint16_t msg_len = strlen((char *)p_string);
		test_msg_size = msg_len;
		if(ble_status == BLE_STATUS_CONNECTED)
		{
			nus_flag = true;
			err_code = ble_nus_c_string_send(&m_ble_nus_c,p_string,msg_len);
			if(err_code != NRF_SUCCESS)
			{

				printf("AT+SENDMSG?\r\n+SENDMSG:send fail\r\nFAIL\r\n");
			}
		}
		else
			printf("AT+SENDMSG?\r\n+SENDMSG:discon\r\nFAIL\r\n");
	}
	else if((length > strlen ("AT+NAME=\r\n"))&&(strncmp((char *)pBuffer,"AT+NAME=",strlen("AT+NAME=")) == 0))
	{
		if(ble_status == BLE_STATUS_CONNECTED)
		{
			sscanf((char *)pBuffer,"AT+NAME=%s",check_target_periph_name);
			if(strlen(check_target_periph_name) > 32)
			{
					printf("AT+NAME=\r\n+NAME:SET  NAME FAIL\r\nFAIL\r\n");
					memset(check_target_periph_name,0,64);	
			}
			else
			{
				memset(set_target_periph_name,0,32);
				sscanf((char *)pBuffer,"AT+NAME=%s",set_target_periph_name);

				err_code = sd_ble_gap_disconnect(m_ble_dfu_c.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
				if(err_code == NRF_SUCCESS)
				{
					err_code =  nrf_ble_scan_all_filter_remove(&m_scan);
					APP_ERROR_CHECK(err_code);

					err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, set_target_periph_name);
					APP_ERROR_CHECK(err_code);

					err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
					APP_ERROR_CHECK(err_code);
					printf("DISCON:OK\r\n");
					if(err_code == NRF_SUCCESS)
					{
						printf("AT+NAME=\r\n+NAME:%s\r\nOK\r\n",set_target_periph_name);
					}
					else
					{
						printf("AT+NAME=\r\n+NAME:SET  NAME FAIL\r\nFAIL\r\n");
				 		memset(set_target_periph_name,0,32);
					}
				}
				else
				{
					printf("AT+NAME=\r\n+NAME:DISCON:FAIL\r\nFAIL\r\n");
					memset(set_target_periph_name,0,32);
				}
				APP_ERROR_CHECK(err_code);
			}
		}
		else if(ble_status == BLE_STATUS_DISCONNECTED)
		{
			sscanf((char *)pBuffer,"AT+NAME=%s",check_target_periph_name);
			if(strlen(check_target_periph_name) > 32)
			{
					printf("AT+NAME=\r\n+NAME:SET  NAME FAIL\r\nFAIL\r\n");
					memset(check_target_periph_name,0,64);	
			}
			else
			{
				memset(set_target_periph_name,0,32);
				sscanf((char *)pBuffer,"AT+NAME=%s",set_target_periph_name);
				err_code =  nrf_ble_scan_all_filter_remove(&m_scan);
				APP_ERROR_CHECK(err_code);

				err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, set_target_periph_name);
				APP_ERROR_CHECK(err_code);

				err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
				APP_ERROR_CHECK(err_code);
				if(err_code == NRF_SUCCESS)
				{
					printf("AT+NAME=\r\n+NAME:%s\r\nOK\r\n",set_target_periph_name);
				}
				else
				{
					printf("AT+NAME=\r\n+NAME:SET NAME FAIL\r\nFAIL\r\n");
					memset(set_target_periph_name,0,32);
				}
			}
		}
	}
	// get scan name
	else if((length ==strlen("AT+NAME?\r\n"))&&(strncmp((char *)pBuffer,"AT+NAME?",strlen("AT+NAME?"))==0))
	{
		int i = 0;
		for(i = 0;i < 32; i++)
		{
			if(set_target_periph_name[i] == 0)
				continue;
			else
			{
				printf("AT+NAME?\r\n+NAME:%s\r\nOK\r\n",set_target_periph_name);
				return;
			}
		}
		printf("AT+NAME?\r\n+NAME:NO NAME SET\r\nOK\r\n");
	}





    //设置连接从机的mac地址
}


/**@brief Function for handling uart rx timeout.
 *
 * @param[in] event_type    Timer event type. 
 * @param[in] p_context     Unused.
 */
void timer_uart_rx_timeout_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    // uint32_t err_code;
    
    nrf_drv_timer_disable(&TIMER_UART_RX);
    
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            NRF_LOG_INFO("UART_RX_STA:%d", UART_RX_STA);
            if(AT_cmd_check_valid(UART_RX_BUF, UART_RX_STA))  // AT command
            {
                AT_cmd_handle(UART_RX_BUF, UART_RX_STA);
            }
			memset(UART_RX_BUF, 0, UART_RX_BUF_SIZE);
            
            UART_RX_STA=0;
            break;

        default:
            //Do nothing.
            break;
    }
}

/**@brief Function for initializing hardware timer.
 */
void timer_uart_rx_timeout_init(void)
{
    uint32_t time_ticks;
    ret_code_t err_code;

    // Define and init timer_cfg struct
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    
    // Init timer
    err_code = nrf_drv_timer_init(  &TIMER_UART_RX, 
                                    &timer_cfg, 
                                    timer_uart_rx_timeout_event_handler);// Register callback   
    APP_ERROR_CHECK(err_code);

    // Convert alarm time(ms) to ticks 
    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_UART_RX, UART_RX_TIMEOUT_INTERVAL);
    
    // Setup timer channel
    nrf_drv_timer_extended_compare( &TIMER_UART_RX, 
                                    NRF_TIMER_CC_CHANNEL0,  // Timer channel 
                                    time_ticks, 
                                    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, 
                                    true);  
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

int main(void)
{
    //初始化地址为0
    memset(set_m_addr, 0, BLE_GAP_ADDR_LEN);

    // Initialize.
    log_init();
    timer_init();
    timer_uart_rx_timeout_init();
    uart_init();
    buttons_leds_init();
    db_discovery_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    dfu_c_init();
	
	nus_c_init();
    scan_init();

    // 打印例程名称
    printf("\r\nAT+RESET:OK\r\n");
    NRF_LOG_INFO("hpy dfu central");
	scan_start();
    // Enter main loop.
    for (;;)
    {
		idle_state_handle();
	}
}
