#include "improv.h"
#include "esp_log.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

improv_wifi_rpc_parsed_command parse_improv_data(const uint8_t *data,
                                                 size_t len) {
  uint8_t command_type = data[0];
  improv_wifi_rpc_parsed_command improv_command;

  // NOTE: current BLE spec for Improv wifi RPC command id only covers 0x01 -
  // 0x02. Change this a newer version of the spec is implemented.
  if (command_type < 0x01 || command_type > 0x02) {
    ESP_LOGE(IMPROV_SDK_TAG, "Command not valid!");

    improv_command.error = IMPROV_ERR_UKNOWN_RPC_CMD;

    return improv_command;
  }

  improv_command.command = (improv_wifi_rpc_command_type)command_type;

  uint8_t data_len = data[1];

  // If the data length is not equal to the actual data (total length - 1 byte
  // for command type - 1 byte for data length - 1 byte for checksum)
  if (data_len != len - 3) {
    improv_command.error = IMPROV_ERR_INVALID_RPC_PACKET;
    return improv_command;
  }

  uint32_t calculated_checksum = 0;
  for (int i = 0; i < len - 1; i++) {
    calculated_checksum += data[i];
  }

  if ((uint8_t)calculated_checksum != data[len - 1]) {
    improv_command.error = IMPROV_ERR_INVALID_RPC_PACKET;
    return improv_command;
  }

  // Special case for wifi settings, we should extract it out from the data
  if (improv_command.command == IMPROV_CMD_WIFI_SETTINGS) {
    uint8_t ssid_len = data[2];
    uint8_t ssid_start = 3;
    size_t ssid_end = ssid_start + ssid_len;

    ESP_LOGI(IMPROV_SDK_TAG,
             "ssid_len: %d, ssid_start: %d, ssid_end: %d, len: %d", ssid_len,
             ssid_start, ssid_end, len);

    if (ssid_end > len) {
      improv_command.error = IMPROV_ERR_INVALID_RPC_PACKET;
      return improv_command;
    }

    uint8_t pass_len = data[ssid_end];
    uint8_t pass_start = ssid_end + 1;
    size_t pass_end = pass_start + pass_len;

    ESP_LOGI(IMPROV_SDK_TAG, "pass_len: %d, pass_start: %d, pass_end: %d",
             pass_len, pass_start, pass_end);

    if (pass_end > len) {
      improv_command.error = IMPROV_ERR_INVALID_RPC_PACKET;
      return improv_command;
    }

    improv_command.ssid = (uint8_t *)malloc(ssid_len * sizeof(uint8_t));
    improv_command.password = (uint8_t *)malloc(pass_len * sizeof(uint8_t));

    // NOTE: might not be the best way to do this
    memcpy(improv_command.ssid, data + ssid_start, ssid_len);
    improv_command.ssid_len = ssid_len;
    memcpy(improv_command.password, data + pass_start, pass_len);
    improv_command.password_len = pass_len;
  }

  improv_command.error = IMPROV_ERR_NO_ERROR;

  return improv_command;
}
