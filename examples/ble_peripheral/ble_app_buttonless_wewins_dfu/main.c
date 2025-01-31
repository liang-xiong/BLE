/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
/** @example examples/ble_peripheral/ble_app_buttonless_dfu
 *
 * @brief Secure DFU Buttonless Service Application main file.
 *
 * This file contains the source code for a sample application using the proprietary
 * Secure DFU Buttonless Service. This is a template application that can be modified
 * to your needs. To extend the functionality of this application, please find
 * locations where the comment "// YOUR_JOB:" is present and read the comments.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "fds.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_power.h"
#include "app_util.h"
#include "nrf_fstorage.h"

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage_sd.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_bootloader_info.h"
#include "nrf_delay.h"
#define DEVICE_NAME                     "NORDIC"                         /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */
#define MANUFACTURER_NAME               "NordicSemiconductor"                       /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define LOGS_UART_DEBUG 0
#define UART_TX_BUF_SIZE                512                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                512                                         /**< UART RX buffer size. */
#define HARDWARE_NUMBER                 ("HW_1_0")
#define FIRMWARE_NUMBER                 ("T_FW_1_0_9")
static uint8_t  UART_RX_BUF[UART_RX_BUF_SIZE] = {0};                                /**< uart receive buffer. */
static uint16_t UART_RX_STA = 0;
static uint16_t UART_RX_FLAG = 0;

static uint8_t  UART_TX_BUF[UART_TX_BUF_SIZE] = {0};                                /**< uart send buffer. */
static uint16_t UART_TX_STA = 0;
static uint16_t UART_TX_FLAG = 0;

static char newname[32] = DEVICE_NAME;

static uint8_t nus_connect = 0;
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                            /**< Handle of the current connection. */
static void advertising_start(bool erase_bonds);                                    /**< Forward declaration of advertising start function */
static void advertising_init(void);
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  //{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x3e000,
    .end_addr   = 0x3ffff,
};

static uint32_t m_data          = 0xBADC0FFE;
static char  __attribute__((aligned)) Name[32] = {"wewins"};
//static char     m_hello_world[] = "hello world";
static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}
/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);


static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};


static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = 0;
}


static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}



// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */

static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

            //收到查询版本事件回复消息
        case BLE_DFU_EVT_GET_VERSION:
            ble_dfu_get_version_resp_send(FIRMWARE_NUMBER, strlen(FIRMWARE_NUMBER));
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{

    // Initialize timer module.
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
	

    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
      =           one.
       uint32_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
}
static void power_manage(void)
{
    (void) sd_app_evt_wait();
}
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        power_manage();
    }
}
/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)newname,
                                          strlen(newname));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
   static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                          ble_yy_service_evt_t * p_evt)
   {
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
   }*/

static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;
		NRF_LOG_INFO("BLE_NUS_EVT_RX_DATA");
        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

		do
        {
//            uint16_t length = (uint16_t)UART_RX_STA;
        	err_code = ble_nus_data_send(&m_nus, (uint8_t *)p_evt->params.rx_data.p_data, &p_evt->params.rx_data.length, m_conn_handle);
            if ((err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != NRF_ERROR_RESOURCES) &&
                (err_code != NRF_ERROR_NOT_FOUND))
            {
                APP_ERROR_CHECK(err_code);
            }
        } while (err_code == NRF_ERROR_RESOURCES);

#if (NRF_LOG_BACKEND_UART_ENABLED == 0)
#if LOGS_UART_DEBUG
        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
//        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
		sprintf((char *)UART_TX_BUF,"ble:received \r\n");
		UART_TX_STA = strlen((char *)UART_TX_BUF);
		UART_TX_FLAG = 1;
#endif
#else
		NRF_LOG_INFO("ble:received( %d ),data :%s,len : %d",nus_connect,p_evt->params.rx_data.p_data,p_evt->params.rx_data.length);
#endif
    }
	else if(p_evt->type == BLE_NUS_EVT_COMM_STARTED)
	{
		NRF_LOG_INFO("BLE_NUS_EVT_COMM_STARTED ");
		nus_connect = 1;
/*
		char *data = "tag's world";
		uint16_t length = strlen(data);
		ble_nus_data_send(&m_nus,(uint8_t *) data, &length, m_conn_handle);
*/
#if (NRF_LOG_BACKEND_UART_ENABLED == 0)
#if LOGS_UART_DEBUG
		sprintf((char *)UART_TX_BUF,"ble:nus_connected\r\n");
		UART_TX_STA = strlen((char *)UART_TX_BUF);
		UART_TX_FLAG = 1;
#endif
#else
		NRF_LOG_INFO("ble:nus_connected( %d ) ",nus_connect);
#endif
	}
	else if (p_evt->type == BLE_NUS_EVT_TX_RDY )
	{
#if (NRF_LOG_BACKEND_UART_ENABLED == 0)
#if LOGS_UART_DEBUG
		sprintf((char *)UART_TX_BUF,"ble:nus_send ok\r\n");
		UART_TX_STA = strlen((char *)UART_TX_BUF);
		UART_TX_FLAG = 1;
#endif
#else
		NRF_LOG_INFO("BLE_NUS_EVT_TX_RDY ");
#endif
	}
	else if (p_evt->type == BLE_NUS_EVT_COMM_STOPPED)
	{
#if (NRF_LOG_BACKEND_UART_ENABLED == 0)
#if LOGS_UART_DEBUG
		sprintf((char *)UART_TX_BUF,"ble:nus_dis_connected\r\n");
		UART_TX_STA = strlen((char *)UART_TX_BUF);
		UART_TX_FLAG = 1;
#endif
#else
		NRF_LOG_INFO("BLE_NUS_EVT_COMM_STOPPED ");
#endif
		nus_connect = 0;
	}


}

/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t                  err_code;
    ble_nus_init_t     nus_init = {0};
    nrf_ble_qwr_init_t        qwr_init  = {0};
    ble_dfu_buttonless_init_t dfus_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

	
	// Initialize DFU.
    dfus_init.evt_handler = ble_dfu_evt_handler;
    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);


	// Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       uint32_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);

    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    //Disable SoftDevice. It is required to be able to write to GPREGRET2 register (SoftDevice API blocks it).
    //GPREGRET2 register holds the information about skipping CRC check on next boot.
    err_code = nrf_sdh_disable_request();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}

int8_t rssi_value = 0;
/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code = NRF_SUCCESS;
	ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    switch (p_ble_evt->header.evt_id)
    {
		case BLE_GAP_EVT_DISCONNECTED:
            // LED indication will be changed when advertising starts.
//			gap_params_init();
//	       	advertising_init();
//	        advertising_start(true);
			rssi_value = 0;
			sd_ble_gap_rssi_stop(m_conn_handle);
            break;

        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
			
            sd_ble_gap_rssi_start(m_conn_handle,0,0);
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
            break;
        }

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
		case BLE_GAP_EVT_RSSI_CHANGED:
			if(rssi_value != p_gap_evt->params.rssi_changed.rssi)
			{
				rssi_value = p_gap_evt->params.rssi_changed.rssi;
				//printf("\r\nrssi= %ddbm\r\n",rssi_value);
//				sprintf((char *)UART_TX_BUF,"rssi= %d dbm\r\n",rssi_value);
//				UART_TX_STA = strlen((char *)UART_TX_BUF);
//				UART_TX_FLAG = 1;
			}
			break;
        default:
            // No implementation needed.
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

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init()
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/** @brief Clear bonding information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}
/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
//	uint32_t err_code;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            //nrf_drv_timer_disable(&TIMER_UART_RX);
            
            UNUSED_VARIABLE(app_uart_get(&UART_RX_BUF[UART_RX_STA]));
            UART_RX_STA++;  // Record the uart received data frame length
            if ((UART_RX_BUF[UART_RX_STA - 1] == '\n') ||
                (UART_RX_STA >= m_ble_nus_max_data_len))
            {
				UART_RX_FLAG = 1;
	//			UART_RX_STA = 0;
			}
            //nrf_drv_timer_enable(&TIMER_UART_RX);
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

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
	int8_t txpwr_level = 0;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	init.advdata.p_tx_power_level		 = &txpwr_level;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    advertising_config_get(&init.config);

    init.evt_handler = on_adv_evt;
    err_code = ble_advertising_init(&m_advertising, &init);
	NRF_LOG_INFO("ble_advertising_init  back :%d ",err_code);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("ble_advertising_conn_cfg_tag_set  before");
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    uint32_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the Power manager.
 */
static void log_init(void)
{
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief   Function for initializing the GATT module.
 * @details The GATT module handles ATT_MTU and Data Length update procedures automatically.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEBUG("advertising is started");
    }
}

static void power_management_init(void)
{
    uint32_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
 
/*
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}
*/

static uint8_t AT_cmd_check_valid(uint8_t *pBuffer, uint16_t length)
{
    // check whether is AT cmd or not   
    if(length < 2) {
        return 0;
    }

    if((strncmp((char*)pBuffer, "AT", 2)!=0)) {
        return 0;
    }

    return 1;
}
static void AT_cmd_handle(uint8_t *pBuffer, uint16_t length)
{

    if((strncmp((char*)pBuffer, "AT?", strlen("AT?")) == 0))
    {
 //       sprintf("AT:OK\r\n"); //初始化进入enter_bootloader_handle回调函数
		sprintf((char *)UART_TX_BUF,"AT:OK\n");
		UART_TX_STA = strlen((char *)UART_TX_BUF);
		UART_TX_FLAG = 1;
    }
    //软件复位
    // System soft reset: AT+RESET\r\n
    else if((length == strlen("AT+RESET\r\n")) && (strncmp((char*)pBuffer, "AT+RESET\r\n", strlen("AT+RESET\r\n")) == 0))
    {
        NVIC_SystemReset(); // Restart the system by default    
    }
    //获取本机版本
    // Hardware/firmware/software version check: AT+VER?\r\n
    else if( (strncmp((char*)pBuffer, "AT+VER?", strlen("AT+VER?")) == 0))
    {
 //       printf("OK\r\nVER:%s\r\n", FIRMWARE_NUMBER);
		sprintf((char *)UART_TX_BUF,"AT+VER?\n+VER:%s\nOK\n", FIRMWARE_NUMBER);
		UART_TX_STA = strlen((char *)UART_TX_BUF);
		UART_TX_FLAG = 1;
    }
	//获取RSSI
	else if(strncmp((char*)pBuffer, "AT+RSSI?", strlen("AT+RSSI?")) == 0)
	{
		if(rssi_value)
			sprintf((char *)UART_TX_BUF,"AT+RSSI?\n+RSSI:%d\nOK\n",rssi_value);
		else
			sprintf((char *)UART_TX_BUF,"AT+RSSI?\n+RSSI:fail\nOK\n");
		UART_TX_STA = strlen((char *)UART_TX_BUF);
		UART_TX_FLAG = 1;
	}
	//连接状态
	else if(strncmp((char*)pBuffer, "AT+STATUS?", strlen("AT+STATUS?")) == 0)
	{
		if(rssi_value)
			sprintf((char *)UART_TX_BUF,"AT+STATUS?\n+STATUS:connected\nOK\n");
		else
			sprintf((char *)UART_TX_BUF,"AT+STATUS?\n+STATUS:disconnected\nOK\n");
		UART_TX_STA = strlen((char *)UART_TX_BUF);
		UART_TX_FLAG = 1;
	}
	//查询本机名字
	else if(strncmp((char*)pBuffer, "AT+NAME?", strlen("AT+NAME?")) == 0)
	{
		uint32_t err_code;
	    err_code = nrf_fstorage_read(&fstorage, 0x3f000, Name, 32);
	    if (err_code != NRF_SUCCESS)
	    {
			sprintf((char *)UART_TX_BUF,"AT+NAME?\nFAIL\n");
			UART_TX_STA = strlen((char *)UART_TX_BUF);
			UART_TX_FLAG = 1;
	    }
		else{
			sprintf((char *)UART_TX_BUF,"AT+NAME?\n+NAME:%s\nOK\n",Name);
			UART_TX_STA = strlen((char *)UART_TX_BUF);
			UART_TX_FLAG = 1;
		}
	}
    //修改本机名字
	else if(strncmp((char*)pBuffer, "AT+NAME=", strlen("AT+NAME=")) == 0)
	{
		if(length > 10) {
		
		 		//先关闭广播
				uint32_t err_code;
				err_code = bsp_indication_set(BSP_INDICATE_IDLE);
				APP_ERROR_CHECK(err_code);
				err_code = (uint32_t) sd_ble_gap_adv_stop(m_advertising.adv_handle);
				APP_ERROR_CHECK(err_code);
			
				//修改名称
//				uint32_t	err_code;
				memset((char *)newname,0,32);
				memcpy((char *)newname,(char *)pBuffer + 8,length - 10);
				ble_gap_conn_sec_mode_t sec_mode;
				BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
				err_code = sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *)newname, length - 10);													
				APP_ERROR_CHECK(err_code);
				advertising_init();
			
				//再开始广播
//				uint32_t err_code;
				err_code = ble_advertising_start(&m_advertising,BLE_ADV_MODE_FAST);
				APP_ERROR_CHECK(err_code);
			
	       	 if(err_code == 0){
			 		memset(Name,0,32);
					memcpy(Name,newname,32);
			 		err_code = nrf_fstorage_erase(&fstorage, 0x3f000, 1, NULL);
				    err_code = nrf_fstorage_write(&fstorage, 0x3f000, Name, 32, NULL);
				    APP_ERROR_CHECK(err_code);
				    wait_for_flash_ready(&fstorage);
				sprintf((char *)UART_TX_BUF,"AT+NAME\n+NAME:%s\nOK\n",(char *)Name);
				UART_TX_STA = strlen((char *)UART_TX_BUF);
				UART_TX_FLAG = 1;
			}
			else
			{
				sprintf((char *)UART_TX_BUF,"AT+NAME\nFAIL\n");
				UART_TX_STA = strlen((char *)UART_TX_BUF);
				UART_TX_FLAG = 1;
			}
		}
		else
		{
			sprintf((char *)UART_TX_BUF,"AT+NAME\nFAIL\n");
			UART_TX_STA = strlen((char *)UART_TX_BUF);
			UART_TX_FLAG = 1;
		}
	}
	else
	{
		strcpy((char *)UART_TX_BUF,(char *)pBuffer);
		UART_TX_STA = strlen((char *)UART_TX_BUF);
		UART_TX_FLAG = 1;
	}
}

static void uart_rx_process(void)
{
    uint32_t       err_code;
	if(AT_cmd_check_valid(UART_RX_BUF,UART_RX_STA) == 0)
	{
	    for (uint32_t i = 0; i < UART_RX_STA; i++)
        {		
			while (app_uart_put(UART_RX_BUF[i]) == NRF_ERROR_BUSY);
       }

	   if(nus_connect && UART_RX_STA >2){
			do
	        {
	            uint16_t length = (uint16_t)UART_RX_STA - 2;
	            err_code = ble_nus_data_send(&m_nus, UART_RX_BUF, &length, m_conn_handle);
	            if ((err_code != NRF_ERROR_INVALID_STATE) &&
	                (err_code != NRF_ERROR_RESOURCES) &&
	                (err_code != NRF_ERROR_NOT_FOUND))
	            {
	                APP_ERROR_CHECK(err_code);
	            }
	        } while (err_code == NRF_ERROR_RESOURCES);	
		}

	}
	else
	{
		AT_cmd_handle(UART_RX_BUF,UART_RX_STA);
	}
	UART_RX_STA = 0;
	memset((char *)UART_RX_BUF,0, UART_RX_BUF_SIZE);
}

static void uart_tx_process(void)
{
    ret_code_t ret_val;
	    for (uint32_t i = 0; i < UART_TX_STA; i++)
        {

		    do
	        {
	            ret_val = app_uart_put(UART_TX_BUF[i]);
	            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
	            {
	                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
	                APP_ERROR_CHECK(ret_val);
	            }
	        } while (ret_val == NRF_ERROR_BUSY);

//			app_uart_put(UART_TX_BUF[i]);
        }
		UART_TX_STA = 0;
		memset((char *)UART_TX_BUF,0, UART_TX_BUF_SIZE);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool       erase_bonds;
	ret_code_t rc;
    ret_code_t err_code;
	NRF_LOG_INFO("main start .");
//	strcpy((char *)newname ,(char *)DEVICE_NAME);
		
#if NRF_LOG_BACKEND_UART_ENABLED
if(0){
	uart_init();
}
#else
	uart_init();
#endif

    log_init();
	timers_init();
	nrf_fstorage_api_t * p_fs_api;
    NRF_LOG_INFO("SoftDevice is present.");
    p_fs_api = &nrf_fstorage_sd;

    rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);
	(void) nrf5_flash_end_addr_get();
	    rc = nrf_fstorage_write(&fstorage, 0x3e000, &m_data, sizeof(m_data), NULL);
    APP_ERROR_CHECK(rc);

    wait_for_flash_ready(&fstorage);
	// Initialize the async SVCI interface to bootloader before any interrupts are enabled.
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);


    power_management_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
	
	    m_data = 0xDEADBEEF;

    NRF_LOG_INFO("Writing \"%x\" to flash.", m_data);
    rc = nrf_fstorage_write(&fstorage, 0x3e100, &m_data, sizeof(m_data), NULL);
    APP_ERROR_CHECK(rc);

    wait_for_flash_ready(&fstorage);

 //   NRF_LOG_INFO("Writing \"%s\" to flash.", m_hello_world);
//	rc = nrf_fstorage_erase(&fstorage, 0x3f000, 1, NULL);
//    rc = nrf_fstorage_write(&fstorage, 0x3f000, Name, sizeof(Name), NULL);
//    APP_ERROR_CHECK(rc);
//    wait_for_flash_ready(&fstorage);
		err_code = nrf_fstorage_read(&fstorage, 0x3f000, Name, 32);
	    if (err_code != NRF_SUCCESS)
	    {
			//memcpy(newname,Name,32);
	    }
		else
		{
			if(Name[0] == 0xFF){
				strcpy((char *)newname ,(char *)DEVICE_NAME);
				memcpy(Name,newname,32);
				err_code = nrf_fstorage_erase(&fstorage, 0x3f000, 1, NULL);
			    err_code = nrf_fstorage_write(&fstorage, 0x3f000, Name, 32, NULL);
			    APP_ERROR_CHECK(err_code);
			    wait_for_flash_ready(&fstorage);
			}
			else
			{
				memcpy(newname,Name,32);
			}
			
		}
	
	NRF_LOG_INFO("peer_manager_init .");
    peer_manager_init();
    gap_params_init();
    gatt_init();
	NRF_LOG_INFO("services_init .");
	services_init();
	NRF_LOG_INFO("advertising_init .");
    advertising_init();
	NRF_LOG_INFO("services_init .");

    conn_params_init();

    NRF_LOG_INFO("Buttonless DFU Application started.");

    // Start execution.
    application_timers_start();
    advertising_start(erase_bonds);



    // Enter main loop.
    for (;;)
    {
//		printf("\r\n------debug-----\r\n");
		if(UART_RX_FLAG == 1)
		{
			uart_rx_process();
			UART_RX_FLAG = 0;			
		}
//		sprintf((char *)UART_TX_BUF,"rssi= %d dbm\r\n",rssi_value);
//		UART_TX_STA = strlen((char *)UART_TX_BUF);
//		UART_TX_FLAG = 1;
		if(UART_TX_FLAG == 1)
		{
			uart_tx_process();
			UART_TX_FLAG = 0;			
		}
//        idle_state_handle();
		//nrf_delay_ms(2000);
    }
}

/**
 * @}
 */
