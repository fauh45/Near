#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#define NEAR_TAG "Near"

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
  // NOTE: Might make the BT/WiFI credentials broken? Although we probably have
  // namespaces, so it won't be too full? Might be broken after flash though, if
  // the size(new) > size(old)
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

  // TODO: initialize GAP and GATTS callback, register app id, set security, etc
  // https://github.com/espressif/esp-idf/blob/v5.3.1/examples/bluetooth/bluedroid/ble/gatt_server_service_table/tutorial/Gatt_Server_Service_Table_Example_Walkthrough.md
}
