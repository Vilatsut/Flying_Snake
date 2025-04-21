# Adafruit Feather Bluefruit Sense with nRF52840

import sys
import time
from math import pi, sqrt

import protocols as p
import ble_uart
import mahony
import ekf
from utils import map_range, measure_vertical_acc, barometer, accelometer, magnetometer, OnboardLEDs

if sys.implementation[0] != 'circuitpython' or sys.implementation[1] !=(9, 2, 7, ""):
    print('Warning: possible CircuitPython version mismatch')
if sys.platform != "nRF52840":
    print("Only Adafruit Feather Bluefruit Sense with nRF52840 supported")

def main():
    FREQUENCY = 200 # Hz
    loop_delay = 1.0 / FREQUENCY

    # Initialize sensors
    baro = barometer()
    accelo = accelometer()
    magneto = magnetometer()

    # Initial magnetometer calibration
    mx, my, mz = magneto.get_magneto_data()
    magneto.calibrate(mx+1, my+1, mz+1) # Cheese, magnetometer values sometimes get stuck -> in init MAGMIN and MAGMAX become the same -> dvision by zero

    # Get initial altitude
    prev_alt = baro.get_altitude()

    # Initialize EKF
    kf = ekf.EKFVariometer(
        dt=1 / FREQUENCY,
        initial_alt=prev_alt,
        accel_noise=0.1,
        accel_z_bias_noise=0.1,
        baro_noise=2.0
    )

    # Initialize Mahony AHRS filter
    ahrs_filter = mahony.Mahony(Kp=4, Ki=0.001, sample_freq=FREQUENCY)

    # Setup LEDs and BLE
    leds = OnboardLEDs()
    leds.red(0)
    leds.rgb()
    ble = ble_uart.MyBLE('Flying Snake')

    print ('Waiting for BLE connection')

    while True:
        leds.rgb(r=10)
        ble.start_advertising()

        # Wait for BLE connection
        while not ble.connected():
            leds.blue(1)
            time.sleep(0.1)
            leds.blue(0)
            time.sleep(0.1)

        # BLE connected
        ble.ble.stop_advertising()
        leds.blue(1)
        leds.rgb(g=10)

        prev_time = time.monotonic()
        while ble.connected():
            start = time.monotonic()

            # Get sensor data
            alt = baro.get_altitude()
            climb = baro.get_vario()

            accel_data = accelo.get_accel_data() # get´accel data in m/s^2
            ax, ay, az = tuple(a*1000/9.80665 for a in accel_data) # turn into milli-gs

            gyro_data = accelo.get_gyro_data() # Get gyro data in radians/second
            gx, gy, gz = tuple(g*180/pi for g in gyro_data) # Change to degrees/second

            mx, my, mz = magneto.get_magneto_data() # Get magnetometer data in microteslas and normalize to [-1,1]
            magneto.calibrate(mx, my, mz) # Update min and max values
            mx = map_range(mx, magneto.MAG_MIN[0], magneto.MAG_MAX[0], -1, 1)
            my = map_range(my, magneto.MAG_MIN[1], magneto.MAG_MAX[1], -1, 1)
            mz = map_range(mz, magneto.MAG_MIN[2], magneto.MAG_MAX[2], -1, 1)

            # Update AHRS orientation
            ahrs_filter.update(-gx, -gy, gz, -ax, -ay, az, -mx, -my, mz)

            # Compute vertical acceleration in m/s2
            vertical_acc = measure_vertical_acc(-ax, -ay, az ,[ahrs_filter.q0, ahrs_filter.q1, ahrs_filter.q2, ahrs_filter.q3])

            # Check accelometer data is in expected range
            # Basic range check for accel data
            accel_mag = sqrt(ax**2 + ay**2 + az**2)
            bUseAccel = 100 < accel_mag < 2000

            # EKF steps
            if bUseAccel:
                kf.predict(vertical_acc)
                kf.update_accel(vertical_acc)
            if alt is not None:
                kf.update_baro(alt)

            # Send BLE data at ~5 Hz
            now = time.monotonic()
            if  now - prev_time > 0.2:
                # Send data at 5 Hz
                elapsed = time.monotonic() - prev_time
                climb = (kf.x[0][0]-prev_alt)/elapsed
                nmea = p.setNmeaShortLXWP0(varioAlt=kf.x[0][0], climbRate=climb)
                ble.write(nmea)
                prev_alt = kf.x[0][0]
                prev_time = now
                print(nmea[:-2])

            # Try to ensure 200Hz is reached
            elapsed = time.monotonic() - start
            sleep_time = loop_delay - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        print ('BLE Disconnected')

main()
