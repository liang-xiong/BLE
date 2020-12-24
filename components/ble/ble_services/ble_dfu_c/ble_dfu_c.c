#include "sdk_common.h"
#include <stdlib.h>

#include "ble.h"
#include "ble_dfu_c.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "app_error.h"

#define NRF_LOG_MODULE_NAME ble_dfu_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/**@brief Function for intercepting the errors of GATTC and the BLE GATT Queue.
 *
 * @param[in] nrf_error   Error code.
 * @param[in] p_ctx       Parameter from the event handler.
 * @param[in] conn_handle Connection handle.
 */
static void gatt_error_handler(uint32_t   nrf_error,
                               void     * p_ctx,
                               uint16_t   conn_handle)
{
    ble_dfu_c_t * p_ble_dfu_c = (ble_dfu_c_t *)p_ctx;

    NRF_LOG_INFO("A GATT Client error has occurred on conn_handle: 0X%X", conn_handle);

    if (p_ble_dfu_c->error_handler != NULL)
    {
        p_ble_dfu_c->error_handler(nrf_error);
    }
}

void ble_dfu_c_on_db_disc_evt(ble_dfu_c_t * p_ble_dfu_c, ble_db_discovery_evt_t * p_evt)
{
    ble_dfu_c_evt_t dfu_c_evt;
    memset(&dfu_c_evt,0,sizeof(ble_dfu_c_evt_t));

    ble_gatt_db_char_t * p_chars = p_evt->params.discovered_db.charateristics;

    // Check if the dfu was discovered.
    if (    (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
        &&  (p_evt->params.discovered_db.srv_uuid.uuid == BLE_DFU_SERVICE_UUID)
        &&  (p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_BLE))
    {
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            switch (p_chars[i].characteristic.uuid.uuid)
            {
                case BLE_DFU_BUTTONLESS_CHAR_UUID:
                    dfu_c_evt.handles.dfu_write_handle = p_chars[i].characteristic.handle_value;
                    dfu_c_evt.handles.dfu_write_cccd_handle = p_chars[i].cccd_handle;
                    break;

                case BLE_DFU_BUTTONLESS_BONDED_CHAR_UUID:
                    dfu_c_evt.handles.dfu_write_handle = p_chars[i].characteristic.handle_value;
                    dfu_c_evt.handles.dfu_write_cccd_handle = p_chars[i].cccd_handle;
                    break;
                case BLE_DFU_CTRL_PT_UUID:
                    NRF_LOG_INFO("BLE_DFU_CTRL_PT_UUID,dfu_write_handle:%d,dfu_write_cccd_handle:%d",p_chars[i].characteristic.handle_value,p_chars[i].cccd_handle);
                    dfu_c_evt.handles.dfu_write_handle = p_chars[i].characteristic.handle_value;
                    dfu_c_evt.handles.dfu_write_cccd_handle = p_chars[i].cccd_handle;
                    break;
                case BLE_DFU_PKT_CHAR_UUID:
                    NRF_LOG_INFO("BLE_DFU_PKT_CHAR_UUID,handle_value:%d,cccd_handle:%d",p_chars[i].characteristic.handle_value,p_chars[i].cccd_handle);
                    dfu_c_evt.handles.dfu_read_handle = p_chars[i].characteristic.handle_value;
                    break;

                default:
                    break;
            }
        }
        if (p_ble_dfu_c->evt_handler != NULL)
        {
            dfu_c_evt.conn_handle = p_evt->conn_handle;
            dfu_c_evt.evt_type    = BLE_DFU_C_EVT_DISCOVERY_COMPLETE;
            p_ble_dfu_c->evt_handler(p_ble_dfu_c, &dfu_c_evt);
        }
    }
}

static void on_hvx(ble_dfu_c_t * p_ble_dfu_c, ble_evt_t const * p_ble_evt)
{
    NRF_LOG_INFO("p_ble_evt->evt.gattc_evt.params.hvx.len:%d", p_ble_evt->evt.gattc_evt.params.hvx.len);
    // HVX can only occur from client sending.
    if (   (p_ble_dfu_c->handles.dfu_write_handle != BLE_GATT_HANDLE_INVALID)
        && (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_dfu_c->handles.dfu_write_handle)
        && (p_ble_dfu_c->evt_handler != NULL))
    {
        ble_dfu_c_evt_t ble_dfu_c_evt;

        ble_dfu_c_evt.evt_type = BLE_DFU_C_EVT_DFU_HVX_EVT;
        ble_dfu_c_evt.p_data   = (uint8_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
        ble_dfu_c_evt.data_len = p_ble_evt->evt.gattc_evt.params.hvx.len;

        p_ble_dfu_c->evt_handler(p_ble_dfu_c, &ble_dfu_c_evt);
    }
}

void on_write_rsp(ble_dfu_c_t * p_ble_dfu_c, ble_evt_t const * p_ble_evt)
{
    NRF_LOG_INFO("dfu_write_handle:%d,BLE_GATT_HANDLE_INVALID: %d", p_ble_dfu_c->handles.dfu_write_handle , BLE_GATT_HANDLE_INVALID);
    NRF_LOG_INFO("write_rsp.handle:%d,dfu_write_handle:%d", p_ble_evt->evt.gattc_evt.params.write_rsp.handle, p_ble_dfu_c->handles.dfu_write_handle);
    NRF_LOG_INFO("p_ble_dfu_c->evt_handler:%d", p_ble_dfu_c->evt_handler);
    NRF_LOG_INFO("dfu_read_handle:%d,write_handle:%d, write_cccd_handle:%d", p_ble_dfu_c->handles.dfu_read_handle,
                                                                                p_ble_dfu_c->handles.dfu_write_handle,
                                                                                p_ble_dfu_c->handles.dfu_write_cccd_handle);
    NRF_LOG_INFO("on_read_rsp_len:%d", p_ble_evt->evt.gattc_evt.params.read_rsp.len);
    NRF_LOG_INFO("on_write_rsp_len:%d", p_ble_evt->evt.gattc_evt.params.write_rsp.len);
    NRF_LOG_INFO("on_hvx_rsp_len:%d", p_ble_evt->evt.gattc_evt.params.hvx.len);
    // if (   (p_ble_dfu_c->handles.dfu_write_handle != BLE_GATT_HANDLE_INVALID)
    //     && (p_ble_evt->evt.gattc_evt.params.write_rsp.handle == p_ble_dfu_c->handles.dfu_write_handle)
    //     && (p_ble_dfu_c->evt_handler != NULL))
    // {
        // ble_dfu_c_evt_t ble_dfu_c_evt;

        // ble_dfu_c_evt.evt_type = BLE_DFU_C_EVT_DFU_WRITE_RSP_EVT;
        // ble_dfu_c_evt.p_data   = (uint8_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
        // ble_dfu_c_evt.data_len = p_ble_evt->evt.gattc_evt.params.hvx.len;
        

        // p_ble_dfu_c->evt_handler(p_ble_dfu_c, &ble_dfu_c_evt);
    // }
}

uint32_t ble_dfu_c_init(ble_dfu_c_t * p_ble_dfu_c, ble_dfu_c_init_t * p_ble_dfu_c_init)
{
    uint32_t      err_code;
    ble_uuid_t    dfu_uuid;
    ble_uuid128_t dfu_base_uuid = BLE_NORDIC_VENDOR_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_ble_dfu_c);
    VERIFY_PARAM_NOT_NULL(p_ble_dfu_c_init);
    VERIFY_PARAM_NOT_NULL(p_ble_dfu_c_init->p_gatt_queue);

    err_code = sd_ble_uuid_vs_add(&dfu_base_uuid, &p_ble_dfu_c->uuid_type);
    VERIFY_SUCCESS(err_code);

    dfu_uuid.type = BLE_UUID_TYPE_BLE;
    dfu_uuid.uuid = BLE_DFU_SERVICE_UUID;

    p_ble_dfu_c->evt_handler           = p_ble_dfu_c_init->evt_handler;
    p_ble_dfu_c->error_handler         = p_ble_dfu_c_init->error_handler;
    p_ble_dfu_c->p_gatt_queue          = p_ble_dfu_c_init->p_gatt_queue;
    p_ble_dfu_c->conn_handle           = BLE_CONN_HANDLE_INVALID;
    p_ble_dfu_c->handles.dfu_write_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_dfu_c->handles.dfu_write_cccd_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_dfu_c->handles.dfu_read_handle = BLE_GATT_HANDLE_INVALID;


    return ble_db_discovery_evt_register(&dfu_uuid);
}

void ble_dfu_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{

    ble_dfu_c_t * p_ble_dfu_c = (ble_dfu_c_t *)p_context;

    if ((p_ble_dfu_c == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    if ( (p_ble_dfu_c->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_dfu_c->conn_handle != p_ble_evt->evt.gap_evt.conn_handle)
       )
    {
        return;
    }

    NRF_LOG_INFO("ble_dfu_c_on_ble_evt:%d", p_ble_evt->header.evt_id);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_dfu_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            if (p_ble_evt->evt.gap_evt.conn_handle == p_ble_dfu_c->conn_handle
                    && p_ble_dfu_c->evt_handler != NULL)
            {
                ble_dfu_c_evt_t dfu_c_evt;

                dfu_c_evt.evt_type = BLE_DFU_C_EVT_DISCONNECTED;

                p_ble_dfu_c->conn_handle = BLE_CONN_HANDLE_INVALID;
                p_ble_dfu_c->evt_handler(p_ble_dfu_c, &dfu_c_evt);
            }
            break;
        case BLE_GATTC_EVT_WRITE_RSP:
            NRF_LOG_INFO("BLE_GATTC_EVT_WRITE_RSP error handle: %x error response %x\r\n",
                                 p_ble_evt->evt.gattc_evt.error_handle,
                                 p_ble_evt->evt.gattc_evt.gatt_status);
            on_write_rsp(p_ble_dfu_c, p_ble_evt);
            break;
        case BLE_GATTS_EVT_TIMEOUT:
            NRF_LOG_INFO("timeout");
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for creating a message for writing to the CCCD. */
static uint32_t cccd_configure(ble_dfu_c_t * p_ble_dfu_c, bool notification_enable)
{
    nrf_ble_gq_req_t cccd_req;
    uint8_t          cccd[BLE_CCCD_VALUE_LEN];
    uint16_t         cccd_val = notification_enable ? BLE_GATT_HVX_INDICATION : 0;

    memset(&cccd_req, 0, sizeof(nrf_ble_gq_req_t));

    cccd[0] = LSB_16(cccd_val);
    cccd[1] = MSB_16(cccd_val);

    cccd_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    cccd_req.error_handler.cb            = gatt_error_handler;
    cccd_req.error_handler.p_ctx         = p_ble_dfu_c;
    cccd_req.params.gattc_write.handle   = p_ble_dfu_c->handles.dfu_write_cccd_handle;
    cccd_req.params.gattc_write.len      = BLE_CCCD_VALUE_LEN;
    cccd_req.params.gattc_write.offset   = 0;
    cccd_req.params.gattc_write.p_value  = cccd;
    cccd_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;
    cccd_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;

    return nrf_ble_gq_item_add(p_ble_dfu_c->p_gatt_queue, &cccd_req, p_ble_dfu_c->conn_handle);
}

/**@brief Function for creating a message for writing to the CCCD. */
static uint32_t cccd_test_configure(ble_dfu_c_t * p_ble_dfu_c, bool notification_enable)
{
    nrf_ble_gq_req_t cccd_req;
    uint8_t          cccd[BLE_CCCD_VALUE_LEN];
    uint16_t         cccd_val = notification_enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    memset(&cccd_req, 0, sizeof(nrf_ble_gq_req_t));

    cccd[0] = LSB_16(cccd_val);
    cccd[1] = MSB_16(cccd_val);

    cccd_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    cccd_req.error_handler.cb            = gatt_error_handler;
    cccd_req.error_handler.p_ctx         = p_ble_dfu_c;
    cccd_req.params.gattc_write.handle   = p_ble_dfu_c->handles.dfu_write_cccd_handle;
    cccd_req.params.gattc_write.len      = BLE_CCCD_VALUE_LEN;
    cccd_req.params.gattc_write.offset   = 0;
    cccd_req.params.gattc_write.p_value  = cccd;
    cccd_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;
    cccd_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;

    return nrf_ble_gq_item_add(p_ble_dfu_c->p_gatt_queue, &cccd_req, p_ble_dfu_c->conn_handle);
}

uint32_t ble_dfu_c_write_notif_enable(ble_dfu_c_t * p_ble_dfu_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_dfu_c);

    if ( (p_ble_dfu_c->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_dfu_c->handles.dfu_write_cccd_handle == BLE_GATT_HANDLE_INVALID)
       )
    {
        return NRF_ERROR_INVALID_STATE;
    }
    return cccd_configure(p_ble_dfu_c, true);
}

uint32_t  ble_dfu_c_write_test_enable(ble_dfu_c_t * p_ble_dfu_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_dfu_c);

    if ( (p_ble_dfu_c->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_dfu_c->handles.dfu_write_cccd_handle == BLE_GATT_HANDLE_INVALID)
       )
    {
        return NRF_ERROR_INVALID_STATE;
    }
    return cccd_test_configure(p_ble_dfu_c, true);
}

uint32_t ble_dfu_c_write_cmd(ble_dfu_c_t * p_ble_dfu_c, uint8_t * p_ble_buf_c, uint16_t length)
{
    VERIFY_PARAM_NOT_NULL(p_ble_dfu_c);

    nrf_ble_gq_req_t write_req;

    memset(&write_req, 0, sizeof(nrf_ble_gq_req_t));

    if (length > BLE_DFU_MAX_DATA_LEN)
    {
        NRF_LOG_WARNING("Content too long.");
        return NRF_ERROR_INVALID_PARAM;
    }
    if (p_ble_dfu_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_WARNING("Connection handle invalid.");
        return NRF_ERROR_INVALID_STATE;
    }

    write_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    write_req.error_handler.cb            = gatt_error_handler;
    write_req.error_handler.p_ctx         = p_ble_dfu_c;
    write_req.params.gattc_write.handle   = p_ble_dfu_c->handles.dfu_read_handle;
    write_req.params.gattc_write.len      = length;
    write_req.params.gattc_write.offset   = 0;
    write_req.params.gattc_write.p_value  = p_ble_buf_c;
    write_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_CMD;
    write_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;

    return nrf_ble_gq_item_add(p_ble_dfu_c->p_gatt_queue, &write_req, p_ble_dfu_c->conn_handle);
}

uint32_t ble_dfu_c_write(ble_dfu_c_t * p_ble_dfu_c, uint8_t * p_ble_buf_c, uint16_t length)
{
    VERIFY_PARAM_NOT_NULL(p_ble_dfu_c);

    nrf_ble_gq_req_t write_req;

    memset(&write_req, 0, sizeof(nrf_ble_gq_req_t));

    if (length > BLE_DFU_MAX_DATA_LEN)
    {
        NRF_LOG_WARNING("Content too long.");
        return NRF_ERROR_INVALID_PARAM;
    }
    if (p_ble_dfu_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_WARNING("Connection handle invalid.");
        return NRF_ERROR_INVALID_STATE;
    }

    NRF_LOG_INFO("ble_dfu_c_write:%d",length);

    ble_gattc_write_params_t const write_params =
    {
        .write_op = BLE_GATT_OP_WRITE_REQ,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = p_ble_dfu_c->handles.dfu_write_handle,
        .offset   = 0,
        .len      = length,
        .p_value  = p_ble_buf_c
    };

    return sd_ble_gattc_write(p_ble_dfu_c->conn_handle, &write_params);

    // write_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    // write_req.error_handler.cb            = gatt_error_handler;
    // write_req.error_handler.p_ctx         = p_ble_dfu_c;
    // write_req.params.gattc_write.handle   = p_ble_dfu_c->handles.dfu_write_handle;
    // write_req.params.gattc_write.len      = length;
    // write_req.params.gattc_write.offset   = 0;
    // write_req.params.gattc_write.p_value  = p_ble_buf_c;
    // write_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;
    // write_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;

    // return nrf_ble_gq_item_add(p_ble_dfu_c->p_gatt_queue, &write_req, p_ble_dfu_c->conn_handle);
}

uint32_t ble_dfu_c_handles_assign(ble_dfu_c_t *               p_ble_dfu,
                                  uint16_t                    conn_handle,
                                  ble_dfu_c_handles_t const * p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_dfu);

    p_ble_dfu->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
        p_ble_dfu->handles.dfu_write_cccd_handle = p_peer_handles->dfu_write_cccd_handle;
        p_ble_dfu->handles.dfu_write_handle      = p_peer_handles->dfu_write_handle;
        p_ble_dfu->handles.dfu_read_handle      = p_peer_handles->dfu_read_handle;
    }
    return nrf_ble_gq_conn_handle_register(p_ble_dfu->p_gatt_queue, conn_handle);
}

uint32_t ble_dfu_get_version_send(ble_dfu_c_t *p_ble_dfu_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_dfu_c);

    nrf_ble_gq_req_t write_req;

    memset(&write_req, 0, sizeof(nrf_ble_gq_req_t));

    if (p_ble_dfu_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_WARNING("Connection handle invalid.");
        return NRF_ERROR_INVALID_STATE;
    }

    uint8_t ble_buf_c = DFU_OP_GET_VERSION;

    ble_gattc_write_params_t const write_params =
    {
        .write_op = BLE_GATT_OP_WRITE_CMD,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = p_ble_dfu_c->handles.dfu_write_handle,
        .offset   = 0,
        .len      = 1,
        .p_value  = &ble_buf_c
    };

    return sd_ble_gattc_write(p_ble_dfu_c->conn_handle, &write_params);
}
