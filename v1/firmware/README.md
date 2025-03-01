# v1 Firmware

The current functionality of v1 is only as such.

1. Onboard the user to connect their WiFi to the device through BLE (or optionally Serial), then ask the user to set their network id
2. Send and receive "nag"s from other device from the same network id
3. Able to update their device through WebSerial

## Ideas

For the WiFi onboarding we could use [Improv WiFI](https://improv-wifi.com). While the device update through WebSerial, we could use [ESP Web Tools](https://esphome.github.io/esp-web-tools/).

While the overall infrastructure for the "nag"s could be defined pretty easily.

```text
Device <-> MQTT <-> Other Device
```

To make sure that each device only connected to the network they want, they could just append their network id on the event name.

## Progress for Beta

- [x] Implement Improv WiFi through BLE
  - [x] Setup GAP & GATT broadcast
  - [x] Setup RPC to handle WiFi connection
  - [x] Add custom RPC to setup network id
- [ ] Setup MQTT connection
  - [x] Handle connection to the server
  - [ ] Handle button press to send event
  - [ ] Handle LED on event listen

## Progress for v1

- [ ] Setup reset system
- [ ] Other improvement we could find on beta?
