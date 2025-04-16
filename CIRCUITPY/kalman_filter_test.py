# Adafruit Feather Bluefruit Sense with nRF52840

import sys
import neopixel
import time
import protocols as p
import board
from digitalio import DigitalInOut, Direction, Pull
from math import pi, sqrt

import kalman_filter
import mahony
import ble_uart
import simple_kalman
from utils import map_range, measure_vertical_acc, HighPassFilter

# Sensor drivers
import adafruit_bmp280
import adafruit_lis3mdl
import adafruit_lsm6ds
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33


if sys.platform != "nRF52840":
    print("Only Adafruit Feather Bluefruit Sense with nRF52840 supported")

if (
    sys.implementation[0] != "circuitpython"
    or sys.implementation[1] != (7, 3, 2)
    or sys.implementation[2] != 517
):
    print("warning, possible version mismatch")


class pressure_sensor:
    def __init__(self):
        self.i2c = board.I2C()
        self.bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)  # temp & baro
        self.bmp280.sea_level_pressure = 1013.25
        self.bmp280.mode = adafruit_bmp280.MODE_NORMAL
        self.bmp280.standby_period = adafruit_bmp280.STANDBY_TC_500
        self.bmp280.iir_filter = adafruit_bmp280.IIR_FILTER_X16
        self.bmp280.overscan_pressure = adafruit_bmp280.OVERSCAN_X16
        self.bmp280.overscan_temperature = adafruit_bmp280.OVERSCAN_X2
        self.prev_altitude = self.bmp280.altitude
        self.prev_time = time.monotonic()

    def get_altitude(self):
        return self.bmp280.altitude

    def get_vario(self):
        altitude_now = self.get_altitude()
        time_now = time.monotonic()
        if time_now - self.prev_time < 0.001:
            time_now = self.prev_time + 0.01
        climb = (altitude_now - self.prev_altitude) / (time_now - self.prev_time)
        self.prev_altitude = altitude_now
        self.prev_time = time_now

        return climb


class accelometer:
    def __init__(self):
        self.i2c = board.I2C()
        self.lsm6ds33 = LSM6DS33(self.i2c)
        self.lsm6ds33.accelerometer_range = adafruit_lsm6ds.AccelRange.RANGE_4G
        self.lsm6ds33.high_pass_filter = adafruit_lsm6ds.AccelHPF.HPF_DIV400
        self.lsm6ds33.accelerometer_data_rate = adafruit_lsm6ds.Rate.RATE_104_HZ

    # Return acceleration data in m / s ^ 2
    def get_accel_data(self):
        return self.lsm6ds33.acceleration

    # Return gyro data in radians / second
    def get_gyro_data(self):
        return self.lsm6ds33.gyro


class magnetometer:
    def __init__(self):
        self.i2c = board.I2C()
        self.lis3md = adafruit_lis3mdl.LIS3MDL(self.i2c)

    def get_magneto_data(self):
        return self.lis3md.magnetic



def main():
    FREQUENCY = 100 # Hz

    hpf = HighPassFilter(0.01, FREQUENCY)

    baro = pressure_sensor()
    accelo = accelometer()
    magneto = magnetometer()

    initial_alt = baro.get_altitude()

    #kf = kalman_filter.KalmanFilter(initial_alt, 0, 0)
    ALT_KF_Q_ACCEL = 1
    ALT_KF_R_PT = 0.1
    alt_estimator = simple_kalman.AltitudeKF(Q_accel=ALT_KF_Q_ACCEL, initial_P=[[1.0, 0.0], [0.0, 1.0]])
    ahrs_filter = mahony.Mahony(1, 0, FREQUENCY)

    prev_time = time.monotonic()
    while True:
        # Get data
        climb = baro.get_vario()
        alt = baro.get_altitude()

        accel_data = accelo.get_accel_data() # get´accel data in m/s^2
        ax, ay, az = tuple(data*1000/9.80665 for data in accel_data) # turn into milli-gs

        gyro_data = accelo.get_gyro_data() # Get gyro data in radians/second
        gx, gy, gz = tuple(data*180/pi for data in gyro_data) # Change to degrees/second

        # Bias Values from calibration
        gx += 0.000996151
        gy -= 0.00190778
        gz -= -0.000713384

        # Values from calibration
        MAG_MIN = [-17.9187, -120.111, -4.67699]
        MAG_MAX = [78.2666, -20.2572, 86.0713]

        # Get magnetometer data in microteslas and normalize to [-1,1]
        mx, my, mz = magneto.get_magneto_data()
        mx = map_range(mx, MAG_MIN[0], MAG_MAX[0], -1, 1)
        my = map_range(my, MAG_MIN[1], MAG_MAX[1], -1, 1)
        mz = map_range(mz, MAG_MIN[2], MAG_MAX[2], -1, 1)

        ahrs_filter.update(-gx, -gy, gz, -ax, -ay, az, mx, my, mz)

        # Vertical acceleration in m/s2
        vertical_acc = hpf.apply(measure_vertical_acc(-ax, -ay, az ,[ahrs_filter.q0, ahrs_filter.q1, ahrs_filter.q2, ahrs_filter.q3]))

        # Check accelometer data is in expected range
        accelMag = sqrt(ax**2 + ay**2 + az**2)
        if 0.6 * 1000 < accelMag < 1.4 * 1000: # 0.6g to 1.4g
            bUseAccel= True
        else:
            bUseAccel = False

        # Kalman filtering
        if bUseAccel:
            alt_estimator.propagate(vertical_acc, 1/FREQUENCY)
        if alt is not None:
            alt_estimator.update(altitude=alt, R_altitude=ALT_KF_R_PT)

        # Send data
        nmea = p.setNmeaShortLXWP0(varioAlt=alt, climbRate=climb)
        #print(nmea[:-2])

        now = time.monotonic()
        if now - prev_time > 1:
            print(f"\nVert_acc: {vertical_acc} Baro alt: {alt} KF alt: {alt_estimator.h}")
            #print("Velocity:", alt_estimator.v)
            print("\nAX: ", ax, " AY: ", ay, " AZ: ", az)
            #print(f"\nQuaternions: {ahrs_filter.q0},{ahrs_filter.q1},{ahrs_filter.q2},{ahrs_filter.q3}")
            #print(f"\nRoll: {ahrs_filter.roll} \t\tYaw: {ahrs_filter.yaw} \t\tPitch: {ahrs_filter.pitch}")
            prev_time = now

        time.sleep(1/FREQUENCY)
