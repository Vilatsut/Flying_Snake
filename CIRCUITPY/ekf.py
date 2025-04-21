import ulab.numpy as np
class EKFVariometer:
    def __init__(self, dt, initial_alt=0, accel_noise=0.2, accel_z_bias_noise=0.001, baro_noise=1.0):
        self.dt = dt

        # State: [altitude, vertical_speed, accel_bias_z]
        self.x = np.array([initial_alt, 0, 0])

        # State covariance
        self.P = np.eye(3) * 5.0

        # Process noise (tune this)
        self.Q = np.diag([accel_noise, accel_noise, accel_z_bias_noise])

        # Measurement noise (barometer)
        self.R_baro = np.array([[baro_noise]])
        self.R_accel = np.array([[accel_noise]])

        # Constant B matrix for control input (acceleration)
        self.B = np.array([
            [0.5 * dt**2],
            [dt],
            [0]
        ])

    def predict(self, z_accel):
        # Build F matrix dynamically in case dt changes
        dt = self.dt
        self.F = np.array([
            [1, dt, -0.5 * dt**2],
            [0, 1, -dt],
            [0, 0, 0.999]
        ])

        self.x = np.dot(self.F, self.x) + self.B * z_accel
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

    def update_baro(self, z_baro):
        H = np.array([[1, 0, 0]])
        z = np.array([[z_baro]])
        self._update(z, H, self.R_baro)

    def update_accel(self, z_accel):
        H = np.array([[0, 0, 1]])
        z = np.array([[z_accel]])
        self._update(z, H, self.R_accel)

    def _update(self, z, H, R):
        y = z - np.dot(H, self.x)
        S = np.dot(H, np.dot(self.P, H.T)) + R
        K = np.dot(self.P, np.dot(H.T, np.linalg.inv(S)))
        self.x = self.x + np.dot(K, y)
        self.P = np.dot((np.eye(3) - np.dot(K, H)), self.P)

    def get_state(self):
        return self.x.flatten()
