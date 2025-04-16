

## Used to calibrate the magenetic sensor by mea
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


def measure_vertical_acc(ax, ay, az, q):
    #print("\nAX: ", ax, "AY: ", ay, "AZ: ", az, "Quaternions: ", q)
    vertical_acc = (
        2.0 * (q[1] * q[3] - q[0] * q[2]) * ax
        + 2.0 * (q[0] * q[1] + q[2] * q[3]) * ay
        + (1 - 2 * (q[1]**2 + q[2]**2)) * az - 1000
    )
    vertical_acc *= 0.00980665  # in m/s**2, assuming ax, ay, az are in milli-Gs
    return vertical_acc

class HighPassFilter:
    def __init__(self, cutoff_frequency, sampling_rate):
        """
        Initialize a High-Pass IIR Filter.

        :param cutoff_frequency: The cutoff frequency of the high-pass filter (Hz).
        :param sampling_rate: The sampling rate of the accelerometer data (Hz).
        """
        # Calculate filter coefficient based on cutoff frequency and sampling rate
        dt = 1 / sampling_rate  # Sampling interval
        rc = 1 / (2 * 3.14159265359 * cutoff_frequency)  # Time constant (RC circuit analogy)
        self.alpha = rc / (rc + dt)

        # Initialize previous input and output values
        self.previous_input = 0.0
        self.previous_output = 0.0

    def apply(self, current_input):
        """
        Apply the high-pass filter to the current accelerometer input.

        :param current_input: the raw accelerometer data.
        :return: A the filtered accelerometer data.
        """
        # Apply the high-pass filter equation: y[n] = α * (y[n-1] + x[n] - x[n-1])
        filtered_output = self.alpha * (self.previous_output + current_input - self.previous_input)


        # Update previous input and output values
        self.previous_input = current_input
        self.previous_output = filtered_output

        return filtered_output
