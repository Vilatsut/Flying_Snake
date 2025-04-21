import time
import board
import neopixel

# Sensor drivers
import adafruit_bmp280
import adafruit_lis3mdl
import adafruit_lsm6ds
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
from digitalio import DigitalInOut

# -------------------------------
# Utility function for mapping
# -------------------------------
def map_range(x, in_min, in_max, out_min, out_max):
    """Map x from one range to another."""
    mapped = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    if out_min <= out_max:
        return max(min(mapped, out_max), out_min)
    return min(max(mapped, out_max), out_min)

# ---------------------------------------------
# Estimate vertical acceleration using quaternion
# ---------------------------------------------
def measure_vertical_acc(ax, ay, az, q):
    """
    Rotate acceleration to vertical frame using quaternion.
    Assumes ax, ay, az are in milli-Gs.
    """
    vertical_acc = (
        2.0 * (q[1] * q[3] - q[0] * q[2]) * ax
        + 2.0 * (q[0] * q[1] + q[2] * q[3]) * ay
        + (1 - 2 * (q[1]**2 + q[2]**2)) * az
        - 1000  # remove gravity
    )
    return vertical_acc * 0.00980665  # convert to m/s²

# -------------------------------
# Barometer Class
# -------------------------------
class barometer:
    def __init__(self):
        self.i2c = board.I2C()
        self.bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)
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
        """Calculate vertical speed from altitude change."""
        altitude_now = self.get_altitude()
        time_now = time.monotonic()

        if time_now - self.prev_time < 0.001:
            time_now = self.prev_time + 0.01

        climb = (altitude_now - self.prev_altitude) / (time_now - self.prev_time)
        self.prev_altitude = altitude_now
        self.prev_time = time_now
        return climb

# -------------------------------
# Accelerometer & Gyro Class
# -------------------------------
class accelometer:
    def __init__(self):
        self.i2c = board.I2C()
        self.lsm6ds33 = LSM6DS33(self.i2c)
        self.lsm6ds33.accelerometer_range = adafruit_lsm6ds.AccelRange.RANGE_4G
        self.lsm6ds33.high_pass_filter = adafruit_lsm6ds.AccelHPF.HPF_DIV400
        self.lsm6ds33.accelerometer_data_rate = adafruit_lsm6ds.Rate.RATE_104_HZ

    def get_accel_data(self):
        """Return acceleration in m/s²."""
        return self.lsm6ds33.acceleration

    def get_gyro_data(self):
        """Return gyro data in rad/s."""
        return self.lsm6ds33.gyro

# -------------------------------
# Magnetometer Class
# -------------------------------
class magnetometer:
    def __init__(self):
        self.i2c = board.I2C()
        self.lis3md = adafruit_lis3mdl.LIS3MDL(self.i2c)
        self.MAG_MIN = [float("inf")] * 3
        self.MAG_MAX = [float("-inf")] * 3

    def get_magneto_data(self):
        return self.lis3md.magnetic

    def calibrate(self, mx, my, mz):
        """Update running min/max for magnetometer calibration."""
        self.MAG_MIN[0] = min(self.MAG_MIN[0], mx)
        self.MAG_MIN[1] = min(self.MAG_MIN[1], my)
        self.MAG_MIN[2] = min(self.MAG_MIN[2], mz)
        self.MAG_MAX[0] = max(self.MAG_MAX[0], mx)
        self.MAG_MAX[1] = max(self.MAG_MAX[1], my)
        self.MAG_MAX[2] = max(self.MAG_MAX[2], mz)

# -------------------------------
# Onboard LED control
# -------------------------------
class OnboardLEDs:
    def __init__(self):
        self.blue_led = DigitalInOut(board.BLUE_LED)
        self.blue_led.switch_to_output()
        self.blue(0)

        self.red_led = DigitalInOut(board.RED_LED)
        self.red_led.switch_to_output()
        self.red(0)

        self.rgbneopixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
        self.rgbneopixel[0] = (0, 0, 0)

    def blue(self, value):
        """Control blue LED."""
        assert value in [0, 1]
        self.blue_led.value = value

    def red(self, value):
        """Control red LED."""
        assert value in [0, 1]
        self.red_led.value = value

    def rgb(self, r=0, g=0, b=0):
        """Set RGB LED color."""
        self.rgbneopixel[0] = (r, g, b)
