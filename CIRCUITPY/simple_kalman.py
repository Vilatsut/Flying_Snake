class AltitudeKF:
    def __init__(self, Q_accel, initial_P):
        # Initialize state
        self.h = 0.0  # Altitude
        self.v = 0.0  # Velocity

        # Initialize state estimate error covariance matrix P
        self.P = initial_P  # Should be a 2x2 matrix

        # Process noise covariance
        self.Q_accel = Q_accel

    def propagate(self, acceleration, dt):
        # Repeated arithmetic
        dtdt = dt * dt

        # Propagation of the state (equation of motion) by Euler integration
        self.h = self.h + self.v * dt + 0.5 * acceleration * dtdt
        self.v = self.v + acceleration * dt

        # Repeated arithmetic for process noise covariance
        Q_accel_dtdt = self.Q_accel * dtdt

        # Calculate the state estimate covariance
        self.P[0][0] += (self.P[1][0] + self.P[0][1] + (self.P[1][1] + 0.25 * Q_accel_dtdt) * dt) * dt
        self.P[0][1] += (self.P[1][1] + 0.5 * Q_accel_dtdt) * dt
        self.P[1][0] += (self.P[1][1] + 0.5 * Q_accel_dtdt) * dt
        self.P[1][1] += Q_accel_dtdt

    def update(self, altitude, R_altitude):
        # Calculate innovation
        y = altitude - self.h

        # Calculate the inverse of the innovation covariance
        Sinv = 1.0 / (self.P[0][0] + R_altitude)

        # Calculate the Kalman gain
        K = [self.P[0][0] * Sinv, self.P[1][0] * Sinv]

        # Update the state estimate
        self.h += K[0] * y
        self.v += K[1] * y

        # Update the state estimate covariance
        P00 = self.P[0][0]
        P01 = self.P[0][1]
        self.P[0][0] -= K[0] * P00
        self.P[0][1] -= K[0] * P01
        self.P[1][0] -= K[1] * P00
        self.P[1][1] -= K[1] * P01

