from collections import namedtuple


# KF4 Acceleration Update variance default
KF_ACCEL_UPDATE_VARIANCE = 50.0

# This is set low as the residual acceleration bias after calibration
# is expected to have little variation/drift
KF_ACCELBIAS_VARIANCE = 0.005
KF_ZMEAS_VARIANCE_DEFAULT = 200 # 0.00255189
KF_ACCEL_VARIANCE_DEFAULT = 100

class State:
    def __init__(self, z=0, v=0, a=0, b=0):
        self.z = z
        self.v = v
        self.a = a
        self.b = b

class KalmanFilter:
    def __init__(self, initialAlt, initialVel, initialAccl):
        self.UseAdaptiveVariance = True

        self.zSensorVariance = KF_ZMEAS_VARIANCE_DEFAULT
        self.AUpdateVariance = KF_ACCEL_UPDATE_VARIANCE*1000
        self.AccelVariance = KF_ACCEL_VARIANCE_DEFAULT*1000
        self.BiasVariance = KF_ACCELBIAS_VARIANCE

        self.State = State(initialAlt, initialVel, initialAccl, 0.0)

        self.Pzz = 1000
        self.Pzv = 0.0
        self.Pza = 0.0
        self.Pzb = 0.0

        self.Pvz = self.Pzv
        self.Pvv = 1000.0
        self.Pva = 0.0
        self.Pvb = 0.0

        self.Paz = self.Pza
        self.Pav = self.Pva
        self.Paa = 50000.0
        self.Pab = 0.0

        self.Pbz = self.Pzb
        self.Pbv = self.Pvb
        self.Pba = self.Pab
        self.Pbb = 1000.0

# Process model state transition matrix F  (4x4)
#  | 1   dt  dt^2/2 -dt^2/2 |
#  | 0   1   dt     -dt     |
#  | 0   0   1       0      |
#  | 0   0   0       1      |
    def predict(self, dt):
        # Predicted (a priori) state vector estimate x_k- = F * x_k-1+
        accel_true = self.State.a - self.State.b # true acceleration = acceleration minus acceleration sensor bias
        self.State.z = self.State.z + (self.State.v * dt) + (accel_true * dt * dt* 0.5)
        self.State.v = self.State.v + (accel_true * dt)

        # Predicted (a priori) state covariance estimate P_k- = (F * P_k-1+ * F_t) + Qk
        dt2 = dt*dt  # dt^2
        dt3 = dt2*dt # dt^3
        dt4 = dt2*dt2 # dt^4;
        dt2div2 = dt2*0.5 # dt^2/2
        dt3div2 = dt3*0.5 # dt^3/2
        dt4div2 = dt4*0.5 # dt^4/2
        dt4div4 = dt4*0.25 # dt^4/4

        p00 = self.Pzz + 2.0*self.Pzv*dt + (self.Pza - self.Pzb)*dt2  + self.Pvv*dt2div2 + (self.Pva - self.Pvb)*dt3 + (self.Paa+self.Pbb)*dt4div4 - self.Pab*dt4div2
        p01 = self.Pzv + dt*(self.Pza - self.Pzb + self.Pvv) + 3.0*dt2div2*(self.Pva - self.Pvb) - self.Pab*dt3 + (self.Paa + self.Pbb)*dt3div2
        p02 = self.Pza + self.Pva*dt + (self.Paa - self.Pba)*dt2div2
        p03 = self.Pzb + self.Pvb*dt + (self.Pab - self.Pbb)*dt2div2

        p11 = self.Pvv + 2.0*dt*(self.Pva - self.Pvb) + dt2*(self.Paa - 2.0*self.Pab + self.Pbb)
        p12 = self.Pva + dt*(self.Paa - self.Pba)
        p13 = self.Pvb + dt*(self.Pab - self.Pbb)

        p22 = self.Paa
        p23 = self.Pab
        p33 = self.Pbb

        self.Pzz = p00
        self.Pzv = p01
        self.Pza = p02
        self.Pzb = p03

        self.Pvz = self.Pzv
        self.Pvv = p11
        self.Pva = p12
        self.Pvb = p13

        self.Paz = self.Pza
        self.Pav = self.Pva
        self.Paa = p22
        self.Pab = p23

        self.Pbz = self.Pzb
        self.Pbv = self.Pvb
        self.Pba = self.Pab
        self.Pbb = p33

        # Add Q_k
        self.Paa = self.Paa + self.AccelVariance
        self.Pbb = self.Pbb + self.BiasVariance

# Update state estimate and state covariance estimate, given new z and a measurements
# Measurement vector m_k
# m_k = | zm_k |
#       | am_k |
# H matrix transforms state vector space to measurement space
# | 1 0 0 0 |
# | 0 0 1 0 |
# Predicted measurement from a_priori state estimate = (H * x_k-)
# Innovation error y_k = actual measurement m_k minus predicted measurement
# y_k = m_k - (H * x_k-)
# 2x2 sensor noise covariance matrix R_k
# | zsensor_variance  0                |
# | 0                 asensor_variance |
    def update(self, zm, am):
        # Innovation Error y_k = measurement minus apriori estimate
        z_err = zm - self.State.z
        a_err = am - self.State.a

        # Innovation covariance S_k
        # S_k = (H * P_k- * H_t) + R_k
        s00 = self.Pzz
        s01 = self.Pza
        s10 = s01
        s11 = self.Paa

        # add R_k
        s00 = s00 + self.zSensorVariance
        if self.UseAdaptiveVariance:
            accel_ext = (am-self.State.b)*(am-self.State.b)
            # allows filter  to respond quickly to moderate/large accelerations while heavily filtering out noise
            # when there is low or no acceleration
            s11 = s11 + accel_ext
            # allow system to update estimated bias only when there is low acceleration
            self.BiasVariance = 1.0/(1.0 + 2.0*accel_ext)

        else:
            s11 = s11 + self.AUpdateVariance;


        # Compute S_k_inv
        sdetinv = 1.0/(s00*s11 - s10*s01)
        sinv00 = sdetinv * s11
        sinv01 = -sdetinv * s10
        sinv10 = sinv01
        sinv11 = sdetinv * s00

        # Kalman gain K_k [4x2] matrix
        # K_k = P_k- * H_t * S_k_inv
        k00 = self.Pzz*sinv00 + self.Pza*sinv10
        k01 = self.Pzz*sinv01 + self.Pza*sinv11
        k10 = self.Pvz*sinv00 + self.Pva*sinv10
        k11 = self.Pvz*sinv01 + self.Pva*sinv11
        k20 = self.Paz*sinv00 + self.Paa*sinv10
        k21 = self.Paz*sinv01 + self.Paa*sinv11
        k30 = self.Pbz*sinv00 + self.Pba*sinv10
        k31 = self.Pbz*sinv01 + self.Pba*sinv11

        # Updated (a posteriori) state estimate x_k+
        # x_k+ = x_k- + K_k * y_k
        self.State.z = self.State.z + (k00*z_err + k01*a_err)
        self.State.v = self.State.v + (k10*z_err + k11*a_err)
        self.State.a = self.State.a + (k20*z_err + k21*a_err)
        self.State.b = self.State.b + (k30*z_err + k31*a_err)

        # Updated (a posteriori) state covariance estimate P_k+
        # P_k+ = (I - K_k * H_k)*P_k-
        tmp = 1.0 - k00
        p00 = tmp*self.Pzz - k01*self.Paz
        p01 = tmp*self.Pzv - k01*self.Pav
        p02 = tmp*self.Pza - k01*self.Paa
        p03 = tmp*self.Pzb - k01*self.Pab

        p11 = -k10*self.Pzv + self.Pvv - k11*self.Pav
        p12 = -k10*self.Pza + self.Pva - k11*self.Paa
        p13 = -k10*self.Pzb + self.Pvb - k11*self.Pab

        p22 = -k20*self.Pza + (1.0-k21)*self.Paa
        p23 = -k20*self.Pzb + (1.0-k21)*self.Pab

        p33 = -k30*self.Pzb -k31*self.Pab + self.Pbb

        self.Pzz = p00
        self.Pzv = p01
        self.Pza = p02
        self.Pzb = p03

        self.Pvz = self.Pzv
        self.Pvv = p11
        self.Pva = p12
        self.Pvb = p13

        self.Paz = self.Pza
        self.Pav = self.Pva
        self.Paa = p22
        self.Pab = p23

        self.Pbz = self.Pzb
        self.Pbv = self.Pvb
        self.Pba = self.Pab
        self.Pbb = p33

        # return the state variables of interest (z and v)
        return (self.State.z, self.State.v)

