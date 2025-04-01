#!/usr/bin/env python3
"""
This script listens to a CAN bus for messages from battery modules.
Each module uses a base address 0x1000 * moduleId.
- Cell data messages are at baseAddress + cell (0-31) and contain:
    struct CellState { uint16_t voltage; uint8_t balancing; }
- ADC raw value messages are at baseAddress + 0x100 + number (0-15) and contain a uint16_t value.
- Module state messages are at baseAddress + 0x200 and contain:
    struct ModuleState {
        uint16_t m1Voltage;
        uint16_t m2Voltage;
        uint16_t cellDiff;
        uint16_t reserved;  // for future use (e.g. temperature)
    }

All values are transmitted in little-endian. Note that the voltage values for cells
are given in 1/10 of mV, so conversion may be applied as needed.

This script decodes the CAN messages and publishes the data via MQTT so that each battery module
is represented as its own device in Home Assistant.
"""

import can
import paho.mqtt.client as mqtt
import struct
import json
import time

# ----- Configuration -----
# MQTT settings
MQTT_BROKER = "192.168.1.2"
MQTT_PORT = 1883
MQTT_TOPIC_PREFIX = "homeassistant/sensor"  # Base topic for Home Assistant sensors

# CAN bus settings
CAN_CHANNEL = "can0"
CAN_BITRATE = 250000  # Adjust this if needed

# ----- Global data store -----
# This dictionary will store the data per module
modules = {}  # key: module_id, value: dict with keys "cells", "adc", and "module"


# ----- MQTT Setup -----
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT Broker with result code:", rc)


mqtt_client = mqtt.Client()
mqtt_client.username_pw_set("gibol", "pixel7")
mqtt_client.on_connect = on_connect
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
mqtt_client.loop_start()


# ----- CAN message processing -----
def process_can_message(msg):
    """
    Process a received CAN message based on its arbitration ID.

    The arbitration ID encodes:
      module_id = arbitration_id // 0x1000
      offset = arbitration_id % 0x1000
    """
    arbitration_id = msg.arbitration_id
    module_id = arbitration_id // 0x1000
    offset = arbitration_id % 0x1000

    # Initialize module storage if not already present
    if module_id not in modules:
        modules[module_id] = {
            "cells": {},  # cell index -> {voltage, balancing}
            "adc": {},    # adc index -> raw value
            "module": {}  # module state info
        }

    # ----- Decode based on offset -----
    if offset < 0x100:
        # This is a cell state message.
        cell_index = offset  # 0-31 expected
        if len(msg.data) >= 3:
            # Unpack little-endian: uint16_t voltage, uint8_t balancing.
            voltage, balancing = struct.unpack("<HB", msg.data[:3])
            # Voltage is in 1/10 of mV so we convert it to mV.
            modules[module_id]["cells"][cell_index] = {
                "voltage": voltage / 10000.0,
                "balancing": balancing
            }
            # print(f"Module {module_id} - Cell {cell_index}: Voltage = {voltage/10.0} mV, Balancing = {balancing}")

    elif 0x100 <= offset < (0x100 + 16):
        # This is an ADC raw value message.
        adc_index = offset - 0x100  # should be 0-15
        if len(msg.data) >= 2:
            (adc_raw,) = struct.unpack("<H", msg.data[:2])
            modules[module_id]["adc"][adc_index] = adc_raw
            # print(f"Module {module_id} - ADC {adc_index}: Raw Value = {adc_raw}")

    elif offset == 0x200:
        # This is a module state message.
        if len(msg.data) >= 8:
            m1Voltage, m2Voltage, cellDiff, reserved = struct.unpack("<HHHH", msg.data[:8])
            modules[module_id]["module"] = {
                "m1Voltage": m1Voltage / 100.0,  # adjust conversion if needed
                "m2Voltage": m2Voltage / 100.0,
                "cellDiff": cellDiff / 10.0,
            }
            # print(f"Module {module_id} - Module State: m1Voltage = {m1Voltage/100.0} V, m2Voltage = {m2Voltage/100.0} V, CellDiff = {cellDiff}")

    else:
        print(f"Module {module_id}: Unknown message offset {hex(offset)}")
        return

    # ----- Publish the updated module state via MQTT -----
    topic = f"{MQTT_TOPIC_PREFIX}/module{module_id}/state"
    payload = json.dumps(modules[module_id])
    mqtt_client.publish(topic, payload)
    print(topic)
    print(payload)
    # print(f"Published updated state for module {module_id} to topic: {topic}")


# ----- Main Loop: Listen on CAN bus -----
def main():
    try:
        # Set up the CAN bus interface
        bus = can.interface.Bus(channel=CAN_CHANNEL, bustype="socketcan", bitrate=CAN_BITRATE)
        print("Listening on CAN bus channel:", CAN_CHANNEL)
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is not None:
                process_can_message(msg)
            else:
                # Optionally, sleep or do other tasks if no message was received
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down...")
    except Exception as e:
        print("Error:", e)
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        print()


if __name__ == "__main__":
    main()
