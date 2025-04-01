import can
import paho.mqtt.client as mqtt
import struct
import json
import time

# Base topic for state updates (each module publishes its complete JSON state here)
MQTT_STATE_TOPIC_PREFIX = "homeassistant/sensor"
# Base topics for discovery (change if desired)
MQTT_CONFIG_TOPIC_PREFIX_SENSOR = "homeassistant/sensor"
MQTT_CONFIG_TOPIC_PREFIX_BINARY_SENSOR = "homeassistant/binary_sensor"

# ----- Configuration -----
# MQTT settings
MQTT_BROKER = "192.168.1.2"
MQTT_PORT = 1883

# CAN bus settings
CAN_CHANNEL = "can0"
CAN_BITRATE = 250000  # Adjust this if needed

# ----- Global Data Store -----
# Each module's data is stored as a dict with keys for cells, adc values, module state,
# and a flag indicating whether the discovery config has been published.
modules = {}  # key: module_id, value: dict with "cells", "adc", "module", "config_published"

# ----- MQTT Setup -----
mqtt_client = mqtt.Client()
# If your broker requires authentication, uncomment and set the credentials:
mqtt_client.username_pw_set("gibol", "pixel7")
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
mqtt_client.loop_start()

def publish_discovery_for_module(module_id):
    """Publish Home Assistant MQTT discovery messages for all sensors for a given module."""
    base_state_topic = f"{MQTT_STATE_TOPIC_PREFIX}/module{module_id}/state"
    device = {
        "identifiers": [f"module{module_id}"],
        "name": f"Module {module_id}"
    }
    # --- 32 Cell Voltage Sensors ---
    for cell in range(32):
        sensor_unique_id = f"module{module_id}_cell_{cell}_voltage"
        topic = f"{MQTT_CONFIG_TOPIC_PREFIX_SENSOR}/{sensor_unique_id}/config"
        payload = {
            "name": f"Module {module_id} Cell {cell} Voltage",
            "state_topic": base_state_topic,
            "value_template": f"{{{{ value_json.cells['{cell}'].voltage_mV }}}}",
            "unit_of_measurement": "mV",
            "device_class": "voltage",
            "unique_id": sensor_unique_id,
            "device": device
        }
        mqtt_client.publish(topic, json.dumps(payload), retain=True)

    # --- 32 Cell Balancing Binary Sensors ---
    for cell in range(32):
        sensor_unique_id = f"module{module_id}_cell_{cell}_balancing"
        topic = f"{MQTT_CONFIG_TOPIC_PREFIX_BINARY_SENSOR}/{sensor_unique_id}/config"
        # The value template converts the balancing value to "ON" or "OFF"
        payload = {
            "name": f"Module {module_id} Cell {cell} Balancing",
            "state_topic": base_state_topic,
            "value_template": f"{{{{ 'ON' if value_json.cells['{cell}'].balancing > 0 else 'OFF' }}}}",
            "payload_on": "ON",
            "payload_off": "OFF",
            "unique_id": sensor_unique_id,
            "device": device
        }
        mqtt_client.publish(topic, json.dumps(payload), retain=True)

    # --- 16 ADC Sensors ---
    for adc in range(16):
        sensor_unique_id = f"module{module_id}_adc_{adc}"
        topic = f"{MQTT_CONFIG_TOPIC_PREFIX_SENSOR}/{sensor_unique_id}/config"
        payload = {
            "name": f"Module {module_id} ADC {adc}",
            "state_topic": base_state_topic,
            "value_template": f"{{{{ value_json.adc['{adc}'] }}}}",
            "unit_of_measurement": "mV",
            "device_class": "voltage",
            "unique_id": sensor_unique_id,
            "device": device
        }
        mqtt_client.publish(topic, json.dumps(payload), retain=True)

    # --- Module Level Sensors ---
    # m1 Voltage Sensor
    sensor_unique_id = f"module{module_id}_m1_voltage"
    topic = f"{MQTT_CONFIG_TOPIC_PREFIX_SENSOR}/{sensor_unique_id}/config"
    payload = {
        "name": f"Module {module_id} M1 Voltage",
        "state_topic": base_state_topic,
        "value_template": "{{ value_json.module.m1Voltage }}",
        "unit_of_measurement": "V",
        "device_class": "voltage",
        "unique_id": sensor_unique_id,
        "device": device
    }
    mqtt_client.publish(topic, json.dumps(payload), retain=True)

    # m2 Voltage Sensor
    sensor_unique_id = f"module{module_id}_m2_voltage"
    topic = f"{MQTT_CONFIG_TOPIC_PREFIX_SENSOR}/{sensor_unique_id}/config"
    payload = {
        "name": f"Module {module_id} M2 Voltage",
        "state_topic": base_state_topic,
        "value_template": "{{ value_json.module.m2Voltage }}",
        "unit_of_measurement": "V",
        "device_class": "voltage",
        "unique_id": sensor_unique_id,
        "device": device
    }
    mqtt_client.publish(topic, json.dumps(payload), retain=True)

    # Cell Diff Sensor
    sensor_unique_id = f"module{module_id}_cell_diff"
    topic = f"{MQTT_CONFIG_TOPIC_PREFIX_SENSOR}/{sensor_unique_id}/config"
    payload = {
        "name": f"Module {module_id} Cell Diff",
        "state_topic": base_state_topic,
        "value_template": "{{ value_json.module.cellDiff }}",
        "unit_of_measurement": "mV",
        "device_class": "voltage",
        "unique_id": sensor_unique_id,
        "device": device
    }
    mqtt_client.publish(topic, json.dumps(payload), retain=True)

    # Current Sensor
    sensor_unique_id = f"module{module_id}_current"
    topic = f"{MQTT_CONFIG_TOPIC_PREFIX_SENSOR}/{sensor_unique_id}/config"
    payload = {
        "name": f"Current",
        "state_topic": base_state_topic,
        "value_template": "{{ value_json.module.current }}",
        "unit_of_measurement": "mA",
        "device_class": "current",
        "unique_id": sensor_unique_id,
        "device": device
    }
    mqtt_client.publish(topic, json.dumps(payload), retain=True)

    print(f"Published discovery config for Module {module_id}")

def process_can_message(msg):
    """
    Process a received CAN message based on its arbitration ID.

    The arbitration ID is assumed to encode:
      module_id = arbitration_id // 0x1000
      offset = arbitration_id % 0x1000
    """
    arbitration_id = msg.arbitration_id
    module_id = arbitration_id // 0x1000
    offset = arbitration_id % 0x1000

    # Initialize module data if not present
    if module_id not in modules:
        modules[module_id] = {
            "cells": {},
            "adc": {},
            "module": {},
            "config_published": False
        }

    # --- Process CAN message ---
    if offset < 0x100:
        # Cell state message: cell index = offset (0-31 expected)
        cell_index = offset
        if len(msg.data) >= 3:
            voltage, balancing = struct.unpack("<HB", msg.data[:3])
            modules[module_id]["cells"][cell_index] = {
                "voltage_mV": voltage / 10.0,
                "balancing": balancing
            }
            print(f"Module {module_id} - Cell {cell_index}: Voltage = {voltage/10.0} mV, Balancing = {balancing}")
    elif 0x100 <= offset < (0x100 + 16):
        # ADC raw value message: adc index = offset - 0x100 (0-15 expected)
        adc_index = offset - 0x100
        if len(msg.data) >= 2:
            (adc_raw,) = struct.unpack("<H", msg.data[:2])
            modules[module_id]["adc"][adc_index] = adc_raw
            print(f"Module {module_id} - ADC {adc_index}: Raw Value = {adc_raw}")
            if adc_index == 7:
                modules[module_id]["module"]["current"] = (adc_raw - 25000) * 1.8
                print(f"Module {module_id}: Current = {(adc_raw - 25000) * 1.8} mA")

    elif offset == 0x200:
        # Module state message
        if len(msg.data) >= 8:
            m1Voltage, m2Voltage, cellDiff, reserved = struct.unpack("<HHHH", msg.data[:8])
            modules[module_id]["module"] = {
                "m1Voltage": m1Voltage / 100.0,
                "m2Voltage": m2Voltage / 100.0,
                "cellDiff": cellDiff / 10.0,
                "reserved": reserved
            }
            print(f"Module {module_id} - Module State: m1Voltage = {m1Voltage/100.0} V, m2Voltage = {m2Voltage/100.0} V, CellDiff = {cellDiff / 10.0}")
    else:
        print(f"Module {module_id}: Unknown offset {hex(offset)}")
        return

    # Publish MQTT discovery configuration for this module (only once)
    if not modules[module_id]["config_published"]:
        publish_discovery_for_module(module_id)
        modules[module_id]["config_published"] = True

    # Publish updated state message (the entire module data) to the state topic
    state_topic = f"{MQTT_STATE_TOPIC_PREFIX}/module{module_id}/state"
    mqtt_client.publish(state_topic, json.dumps(modules[module_id]))
    print(f"Published state for Module {module_id} to {state_topic}")

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
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down...")
    except Exception as e:
        print("Error:", e)
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()

if __name__ == "__main__":
    main()