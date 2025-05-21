"""
MPU6050 Gyroscope & Accelerometer Driver Class
Author: Tomer Ozer

This class provides an interface to the MPU6050 IMU sensor
via I2C. It allows reading raw temperature, acceleration,
and angular velocity (gyroscope) data. It also handles gyro
calibration and integrates angular velocity over time to
compute angular position (θ). Sensitivity constants are set
for ±250°/s and ±2g by default, and can be modified.

Functions:
  - get_temp(): returns temperature in Celsius.
  - get_accel(): returns acceleration in g (gravity units).
  - get_omega(): returns calibrated angular velocity in °/s.
  - get_theta(): returns integrated angle from gyro.
  - reset_theta(): resets the angle.
  - set_gyro_offset(offset): manually set gyro calibration.
  - reset_gyro_offset(): recalibrate gyro to current position.
  
"""


from machine import I2C
import time


class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        
        
        self._PWR_MGMT_1 = 0x6B
        self._TEMP_OUT_H = 0x41
        self._ACCEL_XOUT_H = 0x3B
        self._ACCEL_YOUT_H = 0x3D
        self._ACCEL_ZOUT_H = 0x3F
        self._GYRO_XOUT_H = 0x43
        self._GYRO_YOUT_H = 0x45
        self._GYRO_ZOUT_H = 0x47

        self.i2c.writeto_mem(self.addr, self._PWR_MGMT_1, b'\x00')  # Wake up the MPU6050
        time.sleep(0.1)
        
        self.omega_offset_sensetivity_factor = 131.0  # ±250deg/sec 

        # Sensitivity constants for ω range:
        # ±250°/s = 131.0
        # ±500°/s = 65.5 
        # ±1000°/s = 32.8
        # ±2000°/s = 16.4

        self.accel_offset_sensetivity_factor = 16384.0  #±2g > precision = 2 * range / 2**16 (bits in register) = 0.000061

        # Sensitivity constants for accel range:
        # ±2g = 16384.0          
        # ±4g = 8192.0            
        # ±8g = 4096.0            
        # ±16g = 2048.0

        self.theta = {'x': 0, 'y': 0, 'z': 0}  # θ: int
        self._theta_f = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # θ: float
        self.last_update = time.ticks_ms()
        self.omega_offset = self._calibrate_gyro()  # ω offset

    def _read_raw_data(self, reg):
        high = self.i2c.readfrom_mem(self.addr, reg, 1)[0]
        low = self.i2c.readfrom_mem(self.addr, reg + 1, 1)[0]
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value

    def get_temp(self):
        rawtemp = self._read_raw_data(self._TEMP_OUT_H)
        return rawtemp / 340.0 + 36.53

    def get_accel(self):
        ax = self._read_raw_data(self._ACCEL_XOUT_H) / self.accel_offset_sensetivity_factor
        ay = self._read_raw_data(self._ACCEL_YOUT_H) / self.accel_offset_sensetivity_factor
        az = self._read_raw_data(self._ACCEL_ZOUT_H) / self.accel_offset_sensetivity_factor
        return {'x': ax, 'y': ay, 'z': az}

    def get_omega(self):
        gx = self._read_raw_data(self._GYRO_XOUT_H) / self.omega_offset_sensetivity_factor
        gy = self._read_raw_data(self._GYRO_YOUT_H) / self.omega_offset_sensetivity_factor
        gz = self._read_raw_data(self._GYRO_ZOUT_H) / self.omega_offset_sensetivity_factor
        return {
            'x': gx - self.omega_offset['x'],
            'y': gy - self.omega_offset['y'],
            'z': gz - self.omega_offset['z'],
        }

    def _calibrate_gyro(self, samples=200):
        print("Calibrating gyro (ω), keep device still... ")
        offset = {'x': 0, 'y': 0, 'z': 0}
        for _ in range(samples):
            gx = self._read_raw_data(self._GYRO_XOUT_H) / self.omega_offset_sensetivity_factor
            gy = self._read_raw_data(self._GYRO_YOUT_H) / self.omega_offset_sensetivity_factor
            gz = self._read_raw_data(self._GYRO_ZOUT_H) / self.omega_offset_sensetivity_factor
            offset['x'] += gx
            offset['y'] += gy
            offset['z'] += gz
            time.sleep(0.01)
        offset = {axis: val / samples for axis, val in offset.items()}
        print("Calibration complete. ω offset:", offset)
        return offset

    def update_theta(self):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_update) / 1000  # Δt in seconds
        self.last_update = now

        omega = self.get_omega()
        for axis in ['x', 'y', 'z']:
            self._theta_f[axis] += omega[axis] * dt  # θ += ωΔt
            self.theta[axis] = int(self._theta_f[axis])

    def get_theta(self):
        self.update_theta()
        return self.theta

    def reset_theta(self):
        self.theta = {'x': 0, 'y': 0, 'z': 0}
        self._theta_f = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_update = time.ticks_ms()

    def reset_gyro_offset(self):
        self.omega_offset = self._calibrate_gyro()

    def set_gyro_offset(self, offset):

        if all(axis in offset for axis in ['x', 'y', 'z']):
            self.omega_offset = offset
        else:
            raise ValueError("Offset must include keys: 'x', 'y', and 'z'.")

    def set_gyro_sensitivity(self, factor):
        self.omega_offset_sensetivity_factor = factor

    def set_accel_sensitivity(self, factor):
        self.accel_offset_sensetivity_factor = factor

