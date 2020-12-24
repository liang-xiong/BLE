#ifndef BLE_DFU_C_H__
#define BLE_DFU_C_H__

// #include <stdint.h>
// #include "ble_srv_common.h"
// #include "nrf_sdh_ble.h"

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_gatt.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "nrf_ble_gq.h"
#include "nrf_sdh_ble.h"

#include "sdk_config.h"


#ifdef __cplusplus
extern "C" {
#endif

/**@brief   observer priority.
 * @details Priority of this module's event handler.
 */
#define BLE_DFU_C_BLE_OBSERVER_PRIO   1

/**@brief   Macro for defining a ble_dfu_c instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_DFU_C_DEF(_name)                                                                        \
static ble_dfu_c_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_DFU_C_BLE_OBSERVER_PRIO,                                                   \
                     ble_dfu_c_on_ble_evt, &_name)


#define BLE_DFU_SERVICE_UUID                (0xFE59)
#define BLE_DFU_BUTTONLESS_CHAR_UUID        (0x0003)    /**< Value combined with vendor-specific base to create Unbonded Buttonless characteristic UUID. */
#define BLE_DFU_BUTTONLESS_BONDED_CHAR_UUID (0x0004)    /**< Value combined with vendor-specific base to create Bonded Buttonless characteristic UUID. */
#define BLE_DFU_CTRL_PT_UUID                (0x0001)                      //!< UUID of the DFU Control Point.
#define BLE_DFU_PKT_CHAR_UUID               (0x0002)                       //!< UUID of the DFU Packet Characteristic.

#define OPCODE_LENGTH        1
#define HANDLE_LENGTH        2

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_DFU_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_DFU_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

/**@brief Nordic vendor-specific base UUID.
 */
#define BLE_NORDIC_VENDOR_BASE_UUID                 \
{{                                                  \
    0x50, 0xEA, 0xDA, 0x30, 0x88, 0x83, 0xB8, 0x9F, \
    0x60, 0x4F, 0x15, 0xF3, 0x00, 0x00, 0xC9, 0x8E  \
}}

typedef enum
{
    DFU_RSP_INVALID               = 0x00,                                           /**< Invalid op code. */
    DFU_RSP_SUCCESS               = 0x01,                                           /**< Success. */
    DFU_RSP_OP_CODE_NOT_SUPPORTED = 0x02,                                           /**< Op code not supported. */
    DFU_RSP_OPERATION_FAILED      = 0x04,                                           /**< Operation failed. */
    DFU_RSP_ADV_NAME_INVALID      = 0x05,                                           /**< Requested advertisement name is too short or too long. */
    DFU_RSP_BUSY                  = 0x06,                                           /**< Ongoing async operation. */
    DFU_RSP_NOT_BONDED            = 0x07,                                           /**< Buttonless unavailable due to device not bonded. */
} ble_dfu_buttonless_rsp_code_t;


/**@brief Enumeration of Bootloader DFU Operation codes.
 */
typedef enum
{
    DFU_OP_RESERVED         = 0x00, /**< Reserved for future use. */
    DFU_OP_ENTER_BOOTLOADER = 0x01, /**< Enter bootloader. */
    DFU_OP_SET_ADV_NAME     = 0x02, /**< Set advertisement name to use in DFU mode. */
    DFU_OP_GET_VERSION      = 0x03,
    DFU_OP_RESPONSE_CODE    = 0x20  /**< Response code. */
} ble_dfu_buttonless_op_code_t;

/**@brief Nordic Buttonless DFU Service event type .
 */
typedef enum
{
    BLE_DFU_C_EVT_DISCOVERY_COMPLETE,
    BLE_DFU_C_EVT_DFU_HVX_EVT,
    BLE_DFU_C_EVT_DFU_WRITE_RSP_EVT,           /**< Event indicating that the central received something from a peer. */
    BLE_DFU_C_EVT_DISCONNECTED,    /**< Failure to enter bootloader mode.*/
} ble_dfu_buttonless_c_evt_type_t;

/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t dfu_write_handle;
    uint16_t dfu_write_cccd_handle;
    uint16_t dfu_read_handle;
} ble_dfu_c_handles_t;

/**@brief Structure containing the DFU event data received from the peer. */
typedef struct
{
    ble_dfu_buttonless_c_evt_type_t evt_type;
    uint16_t             conn_handle;
    uint16_t             max_data_len;
    uint8_t            * p_data;
    uint16_t             data_len;
    ble_dfu_c_handles_t  handles;
} ble_dfu_c_evt_t;

// Forward declaration of the ble_dfu_t type.
typedef struct ble_dfu_c_s ble_dfu_c_t;


/**@brief Nordic Buttonless DFU Service event handler type.
 */
typedef void (*ble_dfu_buttonless_c_evt_handler_t) (ble_dfu_c_t * p_ble_dfu_c, ble_dfu_c_evt_t const * p_evt);
/**@brief Enumeration of Bootloader DFU response codes.
 */

/**@brief dfu Client structure. */
struct ble_dfu_c_s
{
    uint8_t                              uuid_type;      /**< UUID type. */
    uint16_t                             conn_handle;    /**< Handle of the current connection. Set with @ref ble_dfu_c_handles_assign when connected. */
    ble_dfu_c_handles_t                  handles;        /**< Handles on the connected peer device needed to interact with it. */
    ble_dfu_buttonless_c_evt_handler_t   evt_handler;    /**< Application event handler to be called when there is an event related to the dfu. */
    ble_srv_error_handler_t              error_handler;  /**< Function to be called in case of an error. */
    nrf_ble_gq_t                         * p_gatt_queue;   /**< Pointer to BLE GATT Queue instance. */
};

typedef struct
{
    ble_dfu_buttonless_c_evt_handler_t   evt_handler;    /**< Application event handler to be called when there is an event related to the dfu. */
    ble_srv_error_handler_t              error_handler;  /**< Function to be called in case of an error. */
    nrf_ble_gq_t                         * p_gatt_queue;   /**< Pointer to BLE GATT Queue instance. */
} ble_dfu_c_init_t;


uint32_t ble_dfu_c_init(ble_dfu_c_t * p_ble_dfu_c, ble_dfu_c_init_t * p_ble_dfu_c_init);

void ble_dfu_c_on_db_disc_evt(ble_dfu_c_t * p_ble_dfu_c, ble_db_discovery_evt_t * p_evt);

void ble_dfu_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

uint32_t ble_dfu_c_write_notif_enable(ble_dfu_c_t * p_ble_dfu_c);
uint32_t ble_dfu_c_write_test_enable(ble_dfu_c_t * p_ble_dfu_c);

uint32_t ble_dfu_c_write_cmd(ble_dfu_c_t * p_ble_dfu_c, uint8_t * p_ble_buf_c, uint16_t length);

uint32_t ble_dfu_c_write(ble_dfu_c_t * p_ble_dfu_c, uint8_t * p_ble_buf_c, uint16_t length);

uint32_t ble_dfu_get_version_send(ble_dfu_c_t *p_ble_dfu_c);

uint32_t ble_dfu_c_handles_assign(ble_dfu_c_t *               p_ble_dfu,
                                  uint16_t                    conn_handle,
                                  ble_dfu_c_handles_t const * p_peer_handles);

#ifdef __cplusplus
}
#endif

#endif // BLE_DIS_H__

/** @} */
