# Hiraya Biogas System (Project BioFlame) ♨️

An IoT-based sensor monitoring system for a continuous-flow, pure cow manure anaerobic biogas digester. This system uses an ESP32 microcontroller to track internal digester conditions, measure methane concentrations, and monitor gas storage capacity in real time.

## 🌟 Features
* **Dual Methane Monitoring:** Calculates CH4 Parts Per Million (PPM) for both the digester environment and the final gas storage.
* **Storage Capacity Alerts:** Uses an ultrasonic sensor to measure gas storage expansion, triggering a buzzer alarm when capacity limits are reached.
* **Environmental Tracking:** Logs precise internal slurry temperatures and pH levels to ensure optimal methanogenesis.
* **Overflow Protection:** Built-in software safeguards to prevent mathematical overflow (`ovf`) errors during sensor burn-in phases.

## 🛠️ Hardware Requirements
* **Microcontroller:** ESP32 Development Board (38-pin version)
* **Temperature:** DS18B20 (Waterproof Digital Temperature Sensor)
* **Gas Sensors:** 2x MQ-4 Methane Sensors
* **pH Sensor:** Analog pH Sensor module
* **Distance/Volume:** HC-SR04 Ultrasonic Sensor
* **Alerts:** Standard 3.3V/5V Active Buzzer

## 📌 ESP32 Pin Configuration
| Component | ESP32 Pin (38-Pin Brd) | Type |
| :--- | :--- | :--- |
| **MQ-4 (Digester)** | `GPIO 36` (VP) | Analog Input |
| **MQ-4 (Storage)** | `GPIO 39` (VN) | Analog Input |
| **pH Sensor** | `GPIO 33` | Analog Input |
| **DS18B20 Temp** | `GPIO 16` | Digital (OneWire) |
| **Ultrasonic TRIG** | `GPIO 17` | Digital Output |
| **Ultrasonic ECHO** | `GPIO 5` | Digital Input |
| **Buzzer** | `GPIO 4` | Digital Output |

## 💻 Software Dependencies
Before compiling this code in the Arduino IDE, ensure you have the ESP32 board manager installed, along with the following libraries:
1. `OneWire` by Paul Stoffregen
2. `DallasTemperature` by Miles Burton

## 🚀 Installation & Setup
1. Open the `.ino` file in the Arduino IDE.
2. Go to **Tools > Board** and select your specific ESP32 module (e.g., *DOIT ESP32 DEVKIT V1*).
3. Ensure the baud rate in your Serial Monitor is set to **115200**.
4. Verify your wiring matches the pin configuration table above. Note that the MQ-4 sensors operate on 5V logic, while the ESP32 analog pins accept a maximum of 3.3V. Ensure proper voltage division or scaling is applied if not using a logic level converter.
5. Compile and upload the code.

## ⚠️ Safety Warning
This system monitors combustible methane gas. Ensure all electronic components, especially the ESP32 and relays, are housed in an intrinsically safe or properly ventilated enclosure isolated from the gas storage to prevent accidental ignition.