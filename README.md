# Smart-Bathroom-Monitor

## Project Overview
The Smart Bathroom Monitoring System is an embedded project developed using the Raspberry Pi Pico W microcontroller, designed to enhance the automation, comfort, and efficiency of a typical bathroom environment. The system integrates multiple sensors, including an infrared (IR) motion sensor and a DHT11 temperature and humidity sensor, to monitor both occupancy and environmental conditions in real time. When a person enters the bathroom, the IR sensor detects motion and triggers the LED indicator and exhaust fan to operate automatically, ensuring both convenience and hygiene. Additionally, the fan can also be activated based on humidity readings, allowing it to respond dynamically to high moisture levels, such as when someone takes a shower. Data such as temperature, humidity, and occupancy status are published to an MQTT broker, enabling remote monitoring and potential integration with other smart home systems.


## Features
- **Automatic Occupancy Detection:** IR motion sensor toggles LED and fan based on bathroom usage.
- **Environmental Monitoring:** DHT11 sensor measures humidity and temperature.
- **MQTT Communication:** Publishes environmental and occupancy data to a cloud MQTT broker.
- **Smart Fan Control:** Fan activates automatically based on humidity threshold or manual MQTT override.
- **Fault-Tolerant Design:** Implements strategies to detect and recover from hardware and software faults.
- **Network Recovery:** Automatically reconnects to Wi-Fi and MQTT after disconnections.


## Hardware Components
| Component | Description |
|------------|-------------|
| Raspberry Pi Pico W | Microcontroller with Wi-Fi support |
| IR Sensor | Detects motion or occupancy |
| DHT11 Sensor | Measures temperature and humidity |
| LED | Visual indicator of occupancy |
| DC Fan | Simulates bathroom exhaust fan |
| (Optional) Transistor/MOSFET | For fan control |
| Resistors & Jumper Wires | Basic circuit connections |
| Breadboard | Prototyping layout |


## Circuit Overview
- The **IR sensor** output pin connects to a digital input on the Pico.
- The **DHT11 sensor** connects to a GPIO pin for digital data.
- The **LED** and **fan** are controlled via GPIO outputs.
- Ensure the fan is powered through a transistor or MOSFET if it draws significant current.


## Software Overview
- Written in **MicroPython** and developed using **Thonny IDE**.
- Utilizes asynchronous programming (`uasyncio`) for non-blocking sensor reading and MQTT communication.
- Uses `mqtt_as` library for lightweight MQTT connectivity.
- Key files:
  - `smart_bathroom_impl.py` or `smart_bathroom_fault_tolerant.py` main system control and MQTT logic.


## MQTT Topics
| Topic | Description |
|--------|-------------|
| `bathroom/occupancy` | Current occupancy status (0/1) |
| `bathroom/humidity` | Humidity percentage |
| `bathroom/temperature` | Temperature in Celsius |
| `bathroom/fan/status` | Current fan state (on/off) |
| `bathroom/fan/set` | Manual override command (on/off/auto) |
| `bathroom/heartbeat` | Periodic system status |
| `bathroom/debug` | Debug and fault-tolerance test messages |


## Fault Tolerance Mechanisms
This project implements multiple fault-tolerance strategies to ensure continuous operation:
- **Hardware Fault Detection:** Identifies when an IR sensor becomes stuck and isolates faulty readings.
- **Software Watchdog Timer:** Automatically resets the system if the main loop becomes unresponsive.
- **Network Reconnection Logic:** Detects Wi-Fi or MQTT disconnections and retries automatically.
- **Debug Monitoring:** Real-time serial and MQTT debug messages track system state and recovery events.


## Testing and Debugging
Extensive testing was performed to validate normal operation and fault-tolerance behavior:
- Simulated **IR sensor failures** confirmed proper fault detection and recovery.
- **Watchdog timer** resets were verified through debug logs.


## How to Run
1. Flash **MicroPython firmware** to the Raspberry Pi Pico W.
2. Open **Thonny IDE** and upload all `.py` files.
3. Update your Wi-Fi SSID, password, and MQTT broker settings in the code.
4. Run `smart_bathroom_impl.py` or `smart_bathroom_fault_tolerant.py` to start the system.


## Future Improvements
- Add mobile dashboard or web interface for real-time monitoring.
- Implement logging to SD card for long-term data storage.
- Integrate sound sensor for gesture-based fan activation.
- Introduce OTA (Over-The-Air) firmware updates.


## License
This project is released under the **MIT License**.  
You are free to use, modify, and distribute it with proper attribution.


## Author
Developed by **Ashkar**  
IoT Systems & Embedded Applications – 2025


