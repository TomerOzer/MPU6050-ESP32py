from machine import Pin, I2C
from Gyro import MPU6050
import time

i = 30
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)

gyro = MPU6050(i2c)

for _ in range(i):
    angle = gyro.get_theta()
    print(angle)
    time.sleep(0.2)

gyro.reset_theta()

for _ in range(i):
    angle = gyro.get_theta()
    print(angle)
    time.sleep(0.2)
