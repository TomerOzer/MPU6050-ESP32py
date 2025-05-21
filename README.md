# MPU6050 MicroPython Class for ESP32

An accurate MicroPython Class for the **MPU6050** 6-axis motion sensor (3-axis gyroscope + 3-axis accelerometer), written specifically for the **ESP32** using `machine.I2C` interface.

---

## 🚀 Features

- 📐 Real-time angle estimation using gyroscope (θ = ∫ω dt)
- 🌡️ Temperature readings
- 📈 Raw acceleration and gyroscope data
- 🧭 Gyroscope auto-calibration
- 🕹️ Compatible with robotics, balancing bots, motion sensing, and more

---

## 📦 Installation

1. Download the class form GitHub.
2. Upload it to your ESP32 using [Thonny](https://thonny.org), or another IDE.
3. Connect the sensor to I2C pins (default: `sda=21`, `scl=22`)

---

## 📄 Usage

```python
from machine import Pin, I2C
from Gyro import MPU6050
import time

i2c = I2C(0, scl=Pin(22), sda=Pin(21))
mpu = MPU6050(i2c)

while True:
    angle = mpu.get_theta()
    temp = mpu.get_temp()
    
    print("Angle:", angle)
    print("Temp:", temp)
    print("--------------------------")
    time.sleep(0.1)
