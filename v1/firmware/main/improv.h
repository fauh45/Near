#pragma once

#include <stddef.h>
#include <stdint.h>

#define IMPROV_SDK_TAG "ImprovSDK"

// Improv WiFi state flags, this explains the current statet the device is in
typedef enum : uint8_t {
  // Awaiting auth through physical interraction
  IMPROV_STATE_AUTH_REQUIRED = 0x01,
  // Ready to accept credentials
  IMPROV_STATE_AUTHORIZED = 0x02,
  // Credentials have been received, attempting to connect
  IMPROV_STATE_PROVISIONING = 0x03,
  // Connection is successful
  IMPROV_STATE_PROVISIONED = 0x04
} improv_wifi_state;

// Improv WiFi possible error state
typedef enum : uint8_t {
  IMPROV_ERR_NO_ERROR = 0x00,
  IMPROV_ERR_INVALID_RPC_PACKET = 0x01,
  IMPROV_ERR_UKNOWN_RPC_CMD = 0x02,
  IMPROV_ERR_UNABLE_TO_CONN = 0x03,
  IMPROV_ERR_NOT_AUTHORIZED = 0x04,
  IMPROV_ERR_UNKNOWN_ERR = 0xFF
} improv_wifi_error_state;

// Improv WiFi RPC command
typedef enum : uint8_t {
  IMPROV_CMD_WIFI_SETTINGS = 0x01,
  IMPROV_CMD_IDENTIFY = 0x02,
} improv_wifi_rpc_command_type;

typedef struct {
  improv_wifi_rpc_command_type command;
  // If not NO_ERROR, throw it to the client
  improv_wifi_error_state error;
  uint8_t *ssid;
  size_t ssid_len;
  uint8_t *password;
  size_t password_len;
} improv_wifi_rpc_parsed_command;

// Parse raw improv data to parsed command struct with checksum check as well
improv_wifi_rpc_parsed_command parse_improv_data(const uint8_t *data,
                                                 size_t len);

// Prepared raw response type with length
typedef struct {
  uint8_t *raw_response;
  size_t response_len;
} improv_prepared_rpc_resp;

// TODO: build a response handler, we don't really needed it now, all the
// response should be empty

// Build a rpc response with added checksum data as well
// improv_prepared_rpc_resp
// build_rpc_response(improv_wifi_rpc_command_type command, const uint8_t *data,
//                    size_t len);
