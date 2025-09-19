# Diagnostic Interface – ESP32-S3  

This module implements a **diagnostic interface** on the ESP32-S3 using a 2.42" SPI OLED display.  
The system shows information from the connected sensors (GNSS, IMU, ENV) as well as from the microcontroller itself (ESP32-S3), organized into multiple pages that can be navigated using a physical button.  

---

## 📟 Functionality  

The interface is designed to be **clear and compact**, splitting data across multiple pages to avoid visual clutter.  

### 🔹 Current Pages  

1. **ENV (BME680)**  
   - Ambient Temperature (°C)  
   - Atmospheric Pressure (hPa)  
   - Humidity (%)  
   - Estimated Altitude (m)  

2. **IMU (MPU6050)**  
   - Acceleration in 3 axes (g)  
   - Gyroscope in 3 axes (°/s)  
   - Sensor Internal Temperature (°C)  

3. **GNSS (NEO-M8N)**  
   GNSS data is divided into **3 pages**:  
   - **GN1:** Speed (kph), Latitude, Longitude, UTC Time  
   - **GN2:** Altitude MSL, Ellipsoid Altitude, Accuracy (2σ), Geoid Separation  
   - **GN3:** Course Over Ground (COG), Magnetic Variation, Motion Status  

4. **SYS1 (ESP32-S3 – MCU Diagnostics)**  
   - CPU Frequency (MHz)  
   - Internal Chip Temperature (°C)  
   - Free / Total Memory (KB)  

5. **SYS2 (ESP32-S3 – Advanced Diagnostics)**  
   - Wi-Fi Signal Strength (RSSI dBm, if connected)  
   - Uptime (seconds since last boot)  
   - Last Reset Cause  

---

## ⚡️ Navigation  

- A **physical button on pin 47** is used to switch pages with each press.  
- Display contents update automatically on each cycle.  

---

## 📂 Project Structure  

- `App/app.py` → Main application controller  
- `App/ui.py` → OLED interface logic (renders each page)  
- `BSW/drivers/` → Sensor wrappers:  
  - `bme680/envsensor.py`  
  - `mpu6050/imusensor.py`  
  - `neo_m8n/gnsssensor.py`  
  - `sysinfo/sysinfosensor.py`  

---

## 🔧 Dependencies  

- **MicroPython** (ESP32-S3 build with SPI + machine + sys support)  
- Custom drivers located in `BSW/drivers`  
- OLED display with SSD1306 controller (SPI version)  

---

## 🚀 Quick Start  

1. **Flash MicroPython**  
   - Download the latest MicroPython firmware for ESP32-S3.  
   - Flash it with `esptool.py`:  
     ```bash
     esptool.py --chip esp32s3 erase_flash
     esptool.py --chip esp32s3 write_flash -z 0x0 firmware.bin
     ```

2. **Install Tools**  
   - Use [Thonny](https://thonny.org/) or `mpremote` to manage and upload files.  

3. **Upload Project Files**  
   - Copy `lib/` (with `App` and `BSW`) into the ESP32 filesystem.  
   - Ensure drivers are inside `lib/BSW/drivers`.  

4. **Run the App**  
   - Place the following in `main.py`:  
     ```python
     from App.app import App
     app = App()
     app.run()
     ```

5. **Power On & Test**  
   - On boot, splash logos will display.  
   - Press the button (pin 47) to cycle through pages.  

---


