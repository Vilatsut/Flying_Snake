import time
import board
import adafruit_lis3mdl
import adafruit_bmp280
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33



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


def calibrate():
    """
    Calibrates a magnometer or gyroscope
    Gyroscope values are in rads/s
    """
    i2c = board.I2C()
    magneto = adafruit_lis3mdl.LIS3MDL(i2c)
    accelo = LSM6DS33(i2c)
    baro = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)


    MAG_MIN = [1000, 1000, 1000]
    MAG_MAX = [-1000, -1000, -1000]

    lastDisplayTime = time.monotonic()
    xavg = 0
    yavg = 0
    zavg = 0
    countavg = 0

    while True:
        x, y, z = magneto.magnetic
        mag_vals = [x, y, z]

        for i in range(3):
            MAG_MIN[i] = min(MAG_MIN[i], mag_vals[i])
            MAG_MAX[i] = max(MAG_MAX[i], mag_vals[i])

        gx, gy, gz = accelo.gyro

        gx = gx * 3.14159 / 180
        xavg += gx
        gy = gy * 3.14159 / 180
        yavg += gy
        gz = gz * 3.14159 / 180
        zavg += gz
        countavg += 1

        if time.monotonic() - lastDisplayTime >= 3:
            print("")
            print("Uncalibrated:", x, y, z)
            cal_x = map_range(x, MAG_MIN[0], MAG_MAX[0], -1, 1)
            cal_y = map_range(y, MAG_MIN[1], MAG_MAX[1], -1, 1)
            cal_z = map_range(z, MAG_MIN[2], MAG_MAX[2], -1, 1)
            print("Calibrated:  ", cal_x, cal_y, cal_z)
            print("MAG_MIN =", MAG_MIN)
            print("MAG_MAX =", MAG_MAX)
            print("Uncalibrated gyro: ", (gx, gy, gz))
            print("Gyro: ", (xavg / countavg, yavg / countavg, zavg / countavg))

            lastDisplayTime = time.monotonic()
