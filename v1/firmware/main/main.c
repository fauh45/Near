#include "esp_bt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"

#include "improv.h"

#include "sdkconfig.h"

#define NEAR_TAG "Near"
#define NEAR_DEVICE_NAME "Near v1"

#define PREPARE_BUF_MAX_SIZE 1024

#define IMPROV_APP_ID 0
#define IMPROV_WIFI_SERVICE_DATA_LEN 6
// How many characteristics does the Improv WiFi have
#define IMPROV_WIFI_CHAR_LEN 5

#define IMPROV_WIFI_GATTS_HANDLE_LEN 20

// Prepare buffer to hold multiple write packets
typedef struct {
  // Buffer data
  uint8_t *prepare_buf;
  // Current offset
  int prepare_len;
} prepare_buf_type_t;

// Prepare buffer for RPC write, that could be multiple packets long
static prepare_buf_type_t rpc_prepare_write_env;

// Flag to indicate which configuration have been done or not
// <- LSB ------------------------------------------------------ MSB ->
// adv_data_config, scan_rsp_config, none, none, none, none, none, none
static uint8_t config_flags = 0;
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

// Service data as required on BT LE advertisement for Improv WiFi
// https://www.improv-wifi.com/ble/ (Service Data format)
//
// <- LSB ----------------------------------------------------- MSB ->
// current state, capabilities, reserved, reserved, reserved, reserved
//
// TODO: might not be correct, an existing example from the ESP Home team
// (https://github.com/esphome/esphome/blob/30477c764d9353381ef6e8bf186bae703cffbc7f/esphome/components/esp32_improv/esp32_improv_component.cpp#L207)
// Also need to figure out how to cycle advertising data, as the standard points
// out: "If the device cannot fit all of its advertising data in 31 bytes, it
// should cycle between advertising data."
static uint8_t improv_wifi_service_data[IMPROV_WIFI_SERVICE_DATA_LEN] = {
    IMPROV_STATE_AUTHORIZED, 0x0, 0x0, 0x0, 0x0, 0x0};

#define IMPROV_STATE improv_wifi_service_data[0]
#define IMPROV_CAPABILITIES improv_wifi_service_data[0]

// Current Improv WiFi error state
static uint8_t improv_wifi_curr_error_state = IMPROV_ERR_NO_ERROR;

// GATT Profile representation, contains the identification, connection, etc
struct gatts_profile_inst {
  // Interface type from the registration event
  uint16_t gatts_if;
  uint16_t app_id;
  uint16_t conn_id;
  uint16_t service_handle;
  // GATTS service identifier representation
  esp_gatt_srvc_id_t service_id;
};

// Characteristics callback type
typedef void (*char_cb_t)(esp_gatts_cb_event_t event,
                          esp_ble_gatts_cb_param_t *param);

// GATT Characteristics representation
struct gatts_char_inst {
  // Characteristics callback on READ | WRITE, depends on the property & perm
  char_cb_t char_cb;
  uint16_t char_handle;
  esp_bt_uuid_t char_uuid;
  esp_gatt_perm_t perm;
  esp_gatt_char_prop_t property;
  uint16_t descr_handle;
  esp_bt_uuid_t descr_uuid;
};

// Equal to "00467768-6228-2272-4663-277478268000"
// (Improv WiFi GATTS Service UUID)
static uint8_t improv_wifi_svc_adv_uuid128[ESP_UUID_LEN_128] = {
    0x00, 0x80, 0x26, 0x78, 0x74, 0x27, 0x63, 0x46,
    0x72, 0x22, 0x28, 0x62, 0x68, 0x77, 0x46, 0x00};

// Store the GATT data as we initialize the profile, do connection, etc
static struct gatts_profile_inst improv_gatts_data = {
    .gatts_if = ESP_GATT_IF_NONE,
    .service_id = {.is_primary = true,
                   .id = {.inst_id = 0x00,
                          // Equal to "00467768-6228-2272-4663-277478268000"
                          // (Improv WiFi GATTS Service UUID)
                          .uuid = {.len = ESP_UUID_LEN_128,
                                   .uuid.uuid128 = {0x00, 0x80, 0x26, 0x78,
                                                    0x74, 0x27, 0x63, 0x46,
                                                    0x72, 0x22, 0x28, 0x62,
                                                    0x68, 0x77, 0x46, 0x00}}}},
};

// Definition for RPC handler after long write event
void handle_rpc();

// Handle capabilities characteristics read event
static void capabilities_char_cb(esp_gatts_cb_event_t event,
                                 esp_ble_gatts_cb_param_t *param) {
  // The response that we're going to send of the read event
  esp_gatt_rsp_t rsp;
  // Set the response to all zero to initialize it
  memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

  rsp.handle = param->read.handle;
  rsp.attr_value.len = 1;
  rsp.attr_value.value[0] = IMPROV_CAPABILITIES;

  // Respond to the read request
  //
  // TODO: if we have more than one service, the gatts if might need to change
  esp_ble_gatts_send_response(improv_gatts_data.gatts_if, param->read.conn_id,
                              param->read.trans_id, ESP_GATT_OK, &rsp);
}

// Handle current state characteristics read event
static void current_state_char_cb(esp_gatts_cb_event_t event,
                                  esp_ble_gatts_cb_param_t *param) {
  // The response that we're going to send of the read event
  esp_gatt_rsp_t rsp;
  // Set the response to all zero to initialize it
  memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

  rsp.handle = param->read.handle;
  rsp.attr_value.len = 1;
  rsp.attr_value.value[0] = IMPROV_STATE;

  // Respond to the read request
  //
  // TODO: if we have more than one service, the gatts if might need to change
  esp_ble_gatts_send_response(improv_gatts_data.gatts_if, param->read.conn_id,
                              param->read.trans_id, ESP_GATT_OK, &rsp);
}

// Handle current error state characteristics read event
static void error_state_char_cb(esp_gatts_cb_event_t event,
                                esp_ble_gatts_cb_param_t *param) {
  // The response that we're going to send of the read event
  esp_gatt_rsp_t rsp;
  // Set the response to all zero to initialize it
  memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

  rsp.handle = param->read.handle;
  rsp.attr_value.len = 1;
  rsp.attr_value.value[0] = improv_wifi_curr_error_state;

  // Respond to the read request
  //
  // TODO: if we have more than one service, the gatts if might need to change
  esp_ble_gatts_send_response(improv_gatts_data.gatts_if, param->read.conn_id,
                              param->read.trans_id, ESP_GATT_OK, &rsp);
}

// Handle RPC command write event
static void rpc_command_char_cb(esp_gatts_cb_event_t event,
                                esp_ble_gatts_cb_param_t *param) {
  esp_gatt_status_t status = ESP_GATT_OK;

  switch (event) {
  case ESP_GATTS_WRITE_EVT: {
    ESP_LOGI(NEAR_TAG, "RPC_COMMAND, value len %d, value :", param->write.len);
    ESP_LOG_BUFFER_HEX(NEAR_TAG, param->write.value, param->write.len);

    if (param->write.need_rsp) {
      // Handle write (non-long) under >23 bytes
      if (!param->write.is_prep) {
        // Copy the while value to buffer for RPC preparation
        memcpy(rpc_prepare_write_env.prepare_buf, param->write.value,
               param->write.len);
        // Add the whole data length
        rpc_prepare_write_env.prepare_len = param->write.len;

        handle_rpc();

        // Free memory after use
        free(rpc_prepare_write_env.prepare_buf);
        rpc_prepare_write_env.prepare_buf = NULL;
        rpc_prepare_write_env.prepare_len = 0;

        esp_ble_gatts_send_response(improv_gatts_data.gatts_if,
                                    param->read.conn_id, param->read.trans_id,
                                    status, NULL);
      }
      // Handle long prep that requires multiple packets
      else {
        // Check if the write offset is bigger than the maximum buffer length
        if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
          status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) >
                   PREPARE_BUF_MAX_SIZE) {
          status = ESP_GATT_INVALID_ATTR_LEN;
        }
        // Initialize the buffer if it has not been used before
        if (status == ESP_GATT_OK &&
            rpc_prepare_write_env.prepare_buf == NULL) {
          rpc_prepare_write_env.prepare_buf =
              (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
          rpc_prepare_write_env.prepare_len = 0;

          // If the buffer is NULL, the malloc have failed, could means there's
          // no more resource in memory
          if (rpc_prepare_write_env.prepare_buf == NULL) {
            ESP_LOGE(NEAR_TAG, "Gatt server prep no mem");
            status = ESP_GATT_NO_RESOURCES;
          }
        }

        // Create temporary response response, each time the long write prep
        // packets arrived, it needs to repond with an ack
        esp_gatt_rsp_t *gatt_rsp =
            (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp) {
          gatt_rsp->attr_value.len = param->write.len;
          gatt_rsp->attr_value.handle = param->write.handle;
          gatt_rsp->attr_value.offset = param->write.offset;
          gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;

          memcpy(gatt_rsp->attr_value.value, param->write.value,
                 param->write.len);
          esp_err_t response_status = esp_ble_gatts_send_response(
              improv_gatts_data.gatts_if, param->write.conn_id,
              param->write.conn_id, status, gatt_rsp);

          if (response_status != ESP_OK) {
            ESP_LOGE(NEAR_TAG, "Send long write response failed\n");
          }
          free(gatt_rsp);
        } else {
          ESP_LOGE(NEAR_TAG,
                   "malloc failed, no resource to send response error\n");
          status = ESP_GATT_NO_RESOURCES;
        }

        // Ignore writing to buffer if the process status is not OK
        if (status != ESP_GATT_OK) {
          return;
        }

        // Copy the value using the offset provided
        memcpy(rpc_prepare_write_env.prepare_buf + param->write.offset,
               param->write.value, param->write.len);
        // Add the offset to the buffer to know the total length
        rpc_prepare_write_env.prepare_len += param->write.len;
      }
    }

    break;
  }

  case ESP_GATTS_EXEC_WRITE_EVT: {
    // Write exec could result in two different confirmation, either confirm or
    // cancel it
    //
    // This will handle the confirmation by executing the RPC
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC) {
      ESP_LOG_BUFFER_HEX(NEAR_TAG, rpc_prepare_write_env.prepare_buf,
                         rpc_prepare_write_env.prepare_len);

      handle_rpc();
    } else {
      // This handle the cancellation of the prep write (long write)
      ESP_LOGI(NEAR_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }

    // If the buffer is not empty, free up the buffer for the next call
    if (rpc_prepare_write_env.prepare_buf) {
      free(rpc_prepare_write_env.prepare_buf);
      rpc_prepare_write_env.prepare_buf = NULL;
    }

    // Reset the buffer lenght to zero
    rpc_prepare_write_env.prepare_len = 0;

    break;
  }

  default:
    break;
  }
}

// Handle rpc result state characteristics read event
static void rpc_result_char_cb(esp_gatts_cb_event_t event,
                               esp_ble_gatts_cb_param_t *param) {
  // TODO: handle actual rpc result

  // The response that we're going to send of the read event
  esp_gatt_rsp_t rsp;
  // Set the response to all zero to initialize it
  memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

  rsp.handle = param->read.handle;
  rsp.attr_value.len = 0;

  // Respond to the read request
  //
  // TODO: if we have more than one service, the gatts if might need to change
  esp_ble_gatts_send_response(improv_gatts_data.gatts_if, param->read.conn_id,
                              param->read.trans_id, ESP_GATT_OK, &rsp);
}

// Which characteristics idx is being initialize current
static int gatts_add_char_init_idx = 0;

// Identifier of all the characteristics available on Improv WiFi
static struct gatts_char_inst improv_gatts_char_data[IMPROV_WIFI_CHAR_LEN] = {
    // "Capabilities" characteristics
    [0] =
        {// Equal to "00467768-6228-2272-4663-277478268005"
         // (Improv WiFi "Capabilities" characteristics UUID)
         .char_uuid = {.len = ESP_UUID_LEN_128,
                       .uuid.uuid128 = {0x05, 0x80, 0x26, 0x78, 0x74, 0x27,
                                        0x63, 0x46, 0x72, 0x22, 0x28, 0x62,
                                        0x68, 0x77, 0x46, 0x00}},
         .property =
             ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
         .perm = ESP_GATT_PERM_READ,
         .char_cb = capabilities_char_cb},
    // "Current State" characteristics
    [1] =
        {// Equal to "00467768-6228-2272-4663-277478268001"
         // (Improv WiFi "Current State" characteristics UUID)
         .char_uuid = {.len = ESP_UUID_LEN_128,
                       .uuid.uuid128 = {0x01, 0x80, 0x26, 0x78, 0x74, 0x27,
                                        0x63, 0x46, 0x72, 0x22, 0x28, 0x62,
                                        0x68, 0x77, 0x46, 0x00}},
         .property =
             ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
         .perm = ESP_GATT_PERM_READ,
         .char_cb = current_state_char_cb},
    // "Error state" characteristics
    [2] =
        {// Equal to "00467768-6228-2272-4663-277478268002"
         // (Improv WiFi "Error State" characteristics UUID)
         .char_uuid = {.len = ESP_UUID_LEN_128,
                       .uuid.uuid128 = {0x02, 0x80, 0x26, 0x78, 0x74, 0x27,
                                        0x63, 0x46, 0x72, 0x22, 0x28, 0x62,
                                        0x68, 0x77, 0x46, 0x0}},
         .property =
             ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
         .perm = ESP_GATT_PERM_READ,
         .char_cb = error_state_char_cb},
    // "RPC Command" characteristics
    [3] =
        {// Equal to "00467768-6228-2272-4663-277478268003"
         // (Improv WiFi "RPC Command" characteristics UUID)
         .char_uuid = {.len = ESP_UUID_LEN_128,
                       .uuid.uuid128 = {0x03, 0x80, 0x26, 0x78, 0x74, 0x27,
                                        0x63, 0x46, 0x72, 0x22, 0x28, 0x62,
                                        0x68, 0x77, 0x46, 0x00}},
         .property = ESP_GATT_CHAR_PROP_BIT_WRITE,
         .perm = ESP_GATT_PERM_WRITE,
         .char_cb = rpc_command_char_cb},
    // "RPC Result" characteristics
    [4] =
        {// Equal to "00467768-6228-2272-4663-277478268004"
         // (Improv WiFi "RPC Result" characteristics UUID)
         .char_uuid = {.len = ESP_UUID_LEN_128,
                       .uuid.uuid128 = {0x04, 0x80, 0x26, 0x78, 0x74, 0x27,
                                        0x63, 0x46, 0x72, 0x22, 0x28, 0x62,
                                        0x68, 0x77, 0x46, 0x00}},
         .property =
             ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
         .perm = ESP_GATT_PERM_READ,
         .char_cb = rpc_result_char_cb},
};

// GATT advertising data, sent to all the surrounding client
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = false,
    .include_txpower = false,
    // Connection intervals are based on 1.25ms units
    .min_interval = 0x6,
    .max_interval = 0x10,
    .appearance = ESP_BLE_APPEARANCE_UNKNOWN,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = sizeof(improv_wifi_service_data),
    .p_service_data = improv_wifi_service_data,
    .service_uuid_len = sizeof(improv_wifi_svc_adv_uuid128),
    .p_service_uuid = improv_wifi_svc_adv_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)};

// GATT response data, as per Improv WiFi standard, the service uuid and the
// service data MUST NOT be in the response data
// https://www.improv-wifi.com/ble/ (Bluetooth LE Advertisement)
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    // Connection intervals are based on 1.25ms units
    .min_interval = 0x6,
    .max_interval = 0x10,
    .appearance = ESP_BLE_APPEARANCE_UNKNOWN,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)};

// Advertising parameters to be used on advertising
//
// https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32c3/api-reference/bluetooth/esp_gap_ble.html?highlight=esp_ble_adv_params_t#_CPPv420esp_ble_adv_params_t
static esp_ble_adv_params_t adv_params = {
    // Advertising interval are on 0.625ms basis
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    // Config to require scans, no connection, etc (assumption)
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY};

// Add the current characteristics idx, if the idx > characteristics number, it
// ignores it
void gatts_add_curr_char_idx() {
  if (gatts_add_char_init_idx < IMPROV_WIFI_CHAR_LEN) {
    esp_ble_gatts_add_char(
        improv_gatts_data.service_handle,
        &improv_gatts_char_data[gatts_add_char_init_idx].char_uuid,
        improv_gatts_char_data[gatts_add_char_init_idx].perm,
        improv_gatts_char_data[gatts_add_char_init_idx].property,
        // TODO: learn what to put here? seems to be optional?
        NULL, NULL);
  }
}

// Handle the Improv WiFi rpc command, and do the action required
//
// All the data are currently taken from the global context, might not be the
// best to do later, but for a quick and dirty implementation should be fine.
void handle_rpc() {
  improv_wifi_rpc_parsed_command parsed_command = parse_improv_data(
      rpc_prepare_write_env.prepare_buf, rpc_prepare_write_env.prepare_len);

  if (parsed_command.error != IMPROV_ERR_NO_ERROR) {
    improv_wifi_curr_error_state = parsed_command.error;

    // TODO: handle notify

    return;
  }

  switch (parsed_command.command) {
  case IMPROV_CMD_IDENTIFY: {
    // NOTE: not currently implemented
    break;
  }

  case IMPROV_CMD_WIFI_SETTINGS: {
    // Remove previously added error
    improv_wifi_curr_error_state = IMPROV_ERR_NO_ERROR;
    // TODO: handle notify

    // TODO: handle wifi connection
    break;
  }

  default:
    break;
  }
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
  switch (event) {
    // This event is triggered when the application profile is registered, with
    // the app_id stored on the param. Need to save the interface type for the
    // profile
  case ESP_GATTS_REG_EVT:
    ESP_LOGI(NEAR_TAG, "REGISTER_APP_EVT, status %d, app_id %d",
             param->reg.status, param->reg.app_id);

    if (param->reg.status == ESP_GATT_OK) {
      improv_gatts_data.gatts_if = gatts_if;

      // Set the device name, might errors out, but we don't really care that
      // much for now
      esp_ble_gap_set_device_name(NEAR_DEVICE_NAME);

      // Set the advertisement packet, this will trigger GAP event
      // "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT"
      esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
      if (ret) {
        ESP_LOGE(NEAR_TAG, "config adv data failed, error code = %x", ret);
      }
      config_flags |= adv_config_flag;

      ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
      if (ret) {
        ESP_LOGE(NEAR_TAG, "config scan response data failed, error code = %x",
                 ret);
      }
      config_flags |= scan_rsp_config_flag;

      esp_ble_gatts_create_service(gatts_if, &improv_gatts_data.service_id,
                                   IMPROV_WIFI_GATTS_HANDLE_LEN);
    } else {
      ESP_LOGI(NEAR_TAG,
               "Reg app profile registration failed, appn id %04x, status %d",
               param->reg.app_id, param->reg.status);
    }
    break;

    // This event is triggered when the gatts create service is called
  case ESP_GATTS_CREATE_EVT:
    ESP_LOGI(NEAR_TAG, "CREATE_SERVICE_EVT, status %d, service_handle %d",
             param->create.status, param->create.service_handle);

    improv_gatts_data.service_handle = param->create.service_handle;

    // Start the service so the GATT client could read if there's this GATSS
    // service
    esp_ble_gatts_start_service(improv_gatts_data.service_handle);

    // Start inititalizing the first characteristics
    gatts_add_curr_char_idx();
    break;

  // This event is triggered when a characteristics have been added
  case ESP_GATTS_ADD_CHAR_EVT:
    ESP_LOGI(NEAR_TAG,
             "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
             param->add_char.status, param->add_char.attr_handle,
             param->add_char.service_handle);

    // If the characteristics is successfully added, save the handles, and go to
    // the next characteristics to initialize
    if (param->add_char.status == ESP_OK) {
      // Save the handle so it could be used on read/write events
      improv_gatts_char_data[gatts_add_char_init_idx].char_handle =
          param->add_char.attr_handle;

      // Add CCC descriptor if notify bit is set, the CCC descriptor standard
      // does call for CCC descriptor for either/both notify and indicate
      // BT_CONTROLLER_INIT_CONFIG_DEFAULT
      //
      // https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/generic-attribute-profile--gatt-.html
      // (Section 3.3.3.3)

      // Save the attributes of the characteristics
      improv_gatts_char_data[gatts_add_char_init_idx].descr_uuid.len =
          ESP_UUID_LEN_16;
      improv_gatts_char_data[gatts_add_char_init_idx].descr_uuid.uuid.uuid16 =
          ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

      esp_ble_gatts_add_char_descr(
          improv_gatts_data.service_handle,
          &improv_gatts_char_data[gatts_add_char_init_idx].descr_uuid,
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
    }

    // NOTE: maybe we should add some descriptor here? might be one way the
    // client able to tell GATTTS server of when to enable or disable
    // notification
    break;

  case ESP_GATTS_ADD_CHAR_DESCR_EVT:
    ESP_LOGI(NEAR_TAG,
             "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
             param->add_char.status, param->add_char.attr_handle,
             param->add_char.service_handle);

    if (param->add_char.status == ESP_OK) {
      // Save the handle for the characteristics descriptor
      improv_gatts_char_data[gatts_add_char_init_idx].descr_handle =
          param->add_char.attr_handle;

      // Continue to initialize the next characteristics
      gatts_add_char_init_idx++;
      gatts_add_curr_char_idx();
    }
    break;

  // This event is triggered when the connected GATT client requesting to read a
  // particular characteristics
  case ESP_GATTS_READ_EVT:
    ESP_LOGI(NEAR_TAG,
             "GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d",
             param->read.conn_id, param->read.trans_id, param->read.handle);

    // Search for the right characteristics by the handle, check the property if
    // it's read, and call the callback to do the read response
    for (int i = 0; i < IMPROV_WIFI_CHAR_LEN; i++) {
      if (param->read.handle == improv_gatts_char_data[i].char_handle &&
          improv_gatts_char_data[i].property & ESP_GATT_CHAR_PROP_BIT_READ) {
        improv_gatts_char_data[i].char_cb(event, param);

        break;
      }
    }

    break;

  // This event is triggered when the connected GATT client requesting to
  // write a particular characteristics/descriptor
  case ESP_GATTS_WRITE_EVT:
    ESP_LOGI(NEAR_TAG,
             "GATT_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d",
             param->write.conn_id, param->write.trans_id, param->write.handle);

    // Handle GATT client activating NOTIFY or INDICATE flags on CCC descriptor,
    // the write request will not be a prep (multi packet write request).
    // NOTE: might need to check this again later
    if (!param->write.is_prep && param->write.len == 2) {
      ESP_LOGI(NEAR_TAG, "GATT_WRITE_EVT (descr), value len %d, value :",
               param->write.len);
      esp_log_buffer_hex(NEAR_TAG, param->write.value, param->write.len);

      uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];

      // Enable NOTIFY bit on descr
      if (descr_value == 0x0001) {
        ESP_LOGI(NEAR_TAG, "notify enable");
      } else if (descr_value == 0x0000) {
        ESP_LOGI(NEAR_TAG, "notify/indicate disable ");
      }

      if (param->write.need_rsp) {
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                    param->write.trans_id, ESP_OK, NULL);
      }
    } else {
      // Search which characteristics for the callback to be called for
      for (int i = 0; i < IMPROV_WIFI_CHAR_LEN; i++) {
        // Check which characteristics are being written on, only works with
        // characteristics that got WRITE bit enabled
        if (param->write.handle == improv_gatts_char_data[i].char_handle &&
            improv_gatts_char_data[i].property & ESP_GATT_CHAR_PROP_BIT_WRITE) {
          improv_gatts_char_data[i].char_cb(event, param);

          break;
        }
      }
    }

    break;

    // This event is triggered on a long write where at the end of the data, the
    // client sends a executive write event to execute the write
  case ESP_GATTS_EXEC_WRITE_EVT:
    ESP_LOGI(NEAR_TAG, "ESP_GATTS_EXEC_WRITE_EVT");

    esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                param->write.trans_id, ESP_GATT_OK, NULL);

    // Search which characteristics for the callback to be called for
    for (int i = 0; i < IMPROV_WIFI_CHAR_LEN; i++) {
      // Check which characteristics are being written on, only works with
      // characteristics that got WRITE bit enabled
      if (param->write.handle == improv_gatts_char_data[i].char_handle &&
          improv_gatts_char_data[i].property & ESP_GATT_CHAR_PROP_BIT_WRITE) {
        improv_gatts_char_data[i].char_cb(event, param);

        break;
      }
    }

    break;

  // This event is triggered when a GATT client is connected to our GATT server
  case ESP_GATTS_CONNECT_EVT:
    ESP_LOGI(NEAR_TAG,
             "ESP_GATTS_CONNECT_EVT, conn_id %d, remote "
             "%02x:%02x:%02x:%02x:%02x:%02x:",
             param->connect.conn_id, param->connect.remote_bda[0],
             param->connect.remote_bda[1], param->connect.remote_bda[2],
             param->connect.remote_bda[3], param->connect.remote_bda[4],
             param->connect.remote_bda[5]);
    improv_gatts_data.conn_id = param->connect.conn_id;
    break;

  // This event is triggered when a GATT client is disconneted to our GATT
  // server
  case ESP_GATTS_DISCONNECT_EVT:
    ESP_LOGI(NEAR_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x",
             param->disconnect.reason);
    // Re-start advertising when the connection is disconneted, on the future,
    // might not do this, instead re-start advertising only after the user have
    // pushed the button?
    esp_ble_gap_start_advertising(&adv_params);
    break;

  default:
    break;
  }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
  switch (event) {
  // Triggered after advertisement data have been sucessfully set
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    config_flags &= (~adv_config_flag);

    ESP_LOGI(NEAR_TAG, "ADV DATA COMPLETE, %d", config_flags);

    // Only start advertising if both advertisement & scan response data have
    // been set
    if (config_flags == 0) {
      esp_ble_gap_start_advertising(&adv_params);
    }
    break;

  // Triggered after scan response data have been sucessfully set
  case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
    config_flags &= (~scan_rsp_config_flag);

    ESP_LOGI(NEAR_TAG, "SCAN RSP DATA COMPLETE, %d", config_flags);

    // Only start advertising if both advertisement & scan response data have
    // been set
    if (config_flags == 0) {
      esp_ble_gap_start_advertising(&adv_params);
    }

    break;

  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    // advertising start complete event to indicate advertising start
    // successfully or failed
    if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(NEAR_TAG, "Advertising start failed");
    }
    break;

  case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
    if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(NEAR_TAG, "Advertising stop failed");
    } else {
      ESP_LOGI(NEAR_TAG, "Stop adv successfully");
    }
    break;

  case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
    ESP_LOGI(
        NEAR_TAG,
        "update connection params status = %d, min_int = %d, max_int = "
        "%d,conn_int = %d,latency = %d, timeout = %d",
        param->update_conn_params.status, param->update_conn_params.min_int,
        param->update_conn_params.max_int, param->update_conn_params.conn_int,
        param->update_conn_params.latency, param->update_conn_params.timeout);
    break;

  case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
    ESP_LOGI(NEAR_TAG, "packet length updated: rx = %d, tx = %d, status = %d",
             param->pkt_data_length_cmpl.params.rx_len,
             param->pkt_data_length_cmpl.params.tx_len,
             param->pkt_data_length_cmpl.status);
    break;

  default:
    break;
  }
}

void app_main(void) {
  printf("Hello world!\n");

  // Temporary variable to see the error response of the function
  esp_err_t res;

  // Inititalize the NVS (non-volatile storage)
  // https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32/api-reference/storage/nvs_flash.html
  res = nvs_flash_init();

  // Check if the NVS have no free space, or there's a newer version, we can
  // safely remove it and retry.
  //
  // NOTE: Might make the BT/WiFI credentials broken? Although we probably
  // have namespaces, so it won't be too full? Might be broken after flash
  // though, if the size(new) > size(old)
  if (res == ESP_ERR_NVS_NO_FREE_PAGES ||
      res == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    res = nvs_flash_init();
  }
  ESP_ERROR_CHECK(res);

  // Clear up controller data used by Classic Bluetooth controller, as we will
  // only use BLE.
  // https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32/api-reference/bluetooth/controller_vhci.html?highlight=esp_bt_controller_mem_release#_CPPv429esp_bt_controller_mem_release13esp_bt_mode_t
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  // Helpful walkthrough on BT and GATT initialization
  // https://github.com/espressif/esp-idf/blob/v5.3.1/examples/bluetooth/bluedroid/ble/gatt_server/tutorial/Gatt_Server_Example_Walkthrough.md

  // Bluetooth controller configuration, mostly configurable from menuconfig
  // https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32/api-reference/bluetooth/controller_vhci.html?highlight=esp_bt_controller_config_t#_CPPv426esp_bt_controller_config_t
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  // Try to initialize the bluetooth controller
  res = esp_bt_controller_init(&bt_cfg);
  if (res) {
    ESP_LOGE(NEAR_TAG, "%s init controller failed: %s", __func__,
             esp_err_to_name(res));
    return;
  }

  // Try to enable the bluetooth controller
  res = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (res) {
    ESP_LOGE(NEAR_TAG, "%s enable bluetooth failed: %s", __func__,
             esp_err_to_name(res));
    return;
  }

  // Initialize and allocate resources for Bluetooth
  // https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32/api-reference/bluetooth/esp_bt_main.html?highlight=esp_bluedroid_init#_CPPv418esp_bluedroid_initv
  res = esp_bluedroid_init();
  if (res) {
    ESP_LOGE(NEAR_TAG, "%s init bluedroind failed: %s", __func__,
             esp_err_to_name(res));

    return;
  }

  // Try to enable bluedroid resources for bluetooth
  res = esp_bluedroid_enable();
  if (res) {
    ESP_LOGE(NEAR_TAG, "%s enable bluedroind failed: %s", __func__,
             esp_err_to_name(res));

    return;
  }

  res = esp_ble_gatts_register_callback(gatts_event_handler);
  if (res) {
    ESP_LOGE(NEAR_TAG, "%s gatts register error: %s", __func__,
             esp_err_to_name(res));

    return;
  }

  res = esp_ble_gap_register_callback(gap_event_handler);
  if (res) {
    ESP_LOGE(NEAR_TAG, "%s gap register error: %s", __func__,
             esp_err_to_name(res));

    return;
  }

  res = esp_ble_gatts_app_register(IMPROV_APP_ID);
  if (res) {
    ESP_LOGE(NEAR_TAG, "%s gatt app register error: %s", __func__,
             esp_err_to_name(res));

    return;
  }

  res = esp_ble_gatt_set_local_mtu(500);
  if (res) {
    ESP_LOGE(NEAR_TAG, "%s gatt set mtu error: %s", __func__,
             esp_err_to_name(res));

    return;
  }

  // TODO: set security, etc
  // https://github.com/espressif/esp-idf/blob/v5.3.1/examples/bluetooth/bluedroid/ble/gatt_server_service_table/tutorial/Gatt_Server_Service_Table_Example_Walkthrough.md
}
