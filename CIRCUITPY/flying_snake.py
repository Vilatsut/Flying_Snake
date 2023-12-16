import time
import protocols as p
import ble_uart
import kalman_filter
# Adafruit Feather Bluefruit Sense with nRF52840
import sys
from math import pi, sqrt

if sys.platform != 'nRF52840':
    print("Only Adafruit Feather Bluefruit Sense with nRF52840 supported")

if sys.implementation[0] != 'circuitpython' or sys.implementation[1] !=(7, 3, 2) or sys.implementation[2]!=517:
    print('warning, possible version mismatch')

# Sensor drivers
import adafruit_bmp280
import adafruit_lis3mdl
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33

import board
from digitalio import DigitalInOut, Direction, Pull

import neopixel

import mahony

class OnboardLEDs:
    def __init__(self):
        self.blue_led = DigitalInOut(board.BLUE_LED)
        self.blue_led.switch_to_output()
        self.blue(0)

        self.red_led = DigitalInOut(board.RED_LED)
        self.red_led.switch_to_output()
        self.red(0)

        # self.switch = DigitalInOut(board.SWITCH)
        # self.red_led.switch_to_input(pull=Pull.UP)

        # Neopixel
        self.rgbneopixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
        self.rgbneopixel[0]=(0,0,0)

    def blue(self, value):
        assert value in [0,1]
        self.blue_led.value = value

    def red(self, value):
        assert value in [0,1]
        self.red_led.value = value

    def rgb(self, r=0,g=0,b=0):
        self.rgbneopixel[0] = (r,g,b)

class pressure_sensor:
    def __init__(self):
        self.i2c = board.I2C()
        self.bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c) # temp & baro
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

    baro = pressure_sensor()
    accelo = accelometer()
    magneto = magnetometer()

    initial_alt = baro.get_altitude()
    kf = kalman_filter.KalmanFilter(initial_alt, 0, 0)

    ahrs_filter = mahony.Mahony(5, 0, 1/0.1)


    leds = OnboardLEDs()
    leds.red(0)
    leds.rgb()

    ble = ble_uart.MyBLE('Flying Snake')

    print ('Waiting for BLE connection')

    while True:
#         leds.rgb(r=10)
#         ble.start_advertising()
#         while not ble.connected():
            #blink blue LED
#             leds.blue(1)
#             time.sleep(0.1)
#             leds.blue(0)
#             time.sleep(0.1)
#         print("BLE Connected")
        #BLE connection established, stop advertising and set blue LED on
#         ble.ble.stop_advertising()
#         leds.blue(1)

        baro.get_vario()
        leds.rgb(g=10)
        while True:#ble.connected():
            # Get data
            climb = baro.get_vario()
            alt = baro.get_altitude()

#             accel_data = accelo.get_accel_data() # Get´accel data in m/s^2 and turn into milli-Gs
#             accel_data = tuple(data*1000/9.81 for data in accel_data)

#             gyro_data = accelo.get_gyro_data() # Get gyro data in radians/second
#             gx, gy, gz = tuple(data*180/pi for data in gyro_data) # Change to degrees/second
#            Bias Values from calibration
#             gx += 0.000996151
#             gy -= 0.00190778
#             gz -= -0.000713384

#            Values from calibration
#             MAG_MIN = [-17.9187, -120.111, -4.67699]
#             MAG_MAX = [78.2666, -20.2572, 86.0713]

#            Get magnetometer data in microteslas
#             mx, my, mz = magneto.get_magneto_data()
#             mx = map_range(mx, MAG_MIN[0], MAG_MAX[0], -1, 1)
#             my = map_range(my, MAG_MIN[1], MAG_MAX[1], -1, 1)
#             mz = map_range(mz, MAG_MIN[2], MAG_MAX[2], -1, 1)

#            Compensate gravity
#             accelMagnitudeSquared = accel_data[0]*accel_data[0] + accel_data[1]*accel_data[1] + accel_data[2]*accel_data[2]
#             if (accelMagnitudeSquared > 562500) and (accelMagnitudeSquared < 1562500):
#                 bUseAccel= True
#             else:
#                 bUseAccel = False

#             ax = accel_data[0]
#             ay = accel_data[1]
#             az = accel_data[2]

#             ahrs_filter.update(-gx, -gy, gz, -ax, -ay, az, mx, my, mz)
#             print(
#                 "Quaternion: ",-gx, -gy, gz, -ax, -ay, az, mx, my, mz
#                 ahrs_filter.q0,
#                 ", ",
#                 ahrs_filter.q1,
#                 ", ",
#                 ahrs_filter.q2,
#                 ", ",
#                 ", ",
#                 ahrs_filter.q3,
#             )
#             print(
#                 "Roll: ",
#                 ahrs_filter.roll,
#                 "\t\tYaw: ",
#                 ahrs_filter.yaw,
#                 "\t\tPitch: ",
#                 ahrs_filter.pitch,
#             )
#            print(temp(-ax, -ay, az ,[ahrs_filter.q0,ahrs_filter.q1,ahrs_filter.q2,ahrs_filter.q3]))
#             print(mx, my, mz) #ax, ay, az, gx, gy, gz, mx, my, mz

#             Kalman filtering

            # Send data
            nmea = p.setNmeaShortLXWP0(varioAlt=alt,climbRate=climb)
            ble.write(nmea)
            ble.write(nmea)
            print(nmea[:-2])
#             yaw = ahrs_filter.yaw * 57.20578
#             if yaw < 0:  # adjust yaw to be between 0 and 360
#                 yaw += 360
#             print(
#                 "Orientation: ",
#                 yaw,
#                 ", ",
#                 ahrs_filter.pitch * 57.29578,
#                 ", ",
#                 ahrs_filter.roll * 57.29578,
#             )
#             print(
#                 "Quaternion: ",
#                 ahrs_filter.q0,
#                 ", ",
#                 ahrs_filter.q1,
#                 ", ",
#                 ahrs_filter.q2,
#                 ", ",
#                 ahrs_filter.q3,
#             )

            # 5Hz
            time.sleep(0.1)
        print ('BLE Disconnected')

def temp(ax, ay, az, q):
    print("AX: ", ax)
    acc = 2.0*(q[0]*q[3] - q[0]*q[2])*ax + 2.0*(q[0]*q[1] + q[2]*q[3])*ay + (q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])*az -1000
    acc *= 0.98 # in cm/s/s, assuming ax, ay, az are in milli-Gs
    return acc

## Used to calibrate the magenetic sensor
def map_range(x, in_min, in_max, out_min, out_max):
    """
    Maps a number from one range to another.
    :return: Returns value mapped to new range
    :rtype: float
    """
    mapped = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    if out_min <= out_max:
        return max(min(mapped, out_max), out_min)

    return min(max(mapped, out_max), out_min)
