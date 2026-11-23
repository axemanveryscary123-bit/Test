class PIDController:
  """This implements a classic closed-loop PID Controller algorithm for one dimension."""
  def __init__(self, kp, ki, kd, dt, y = 0, minu = -float("inf"), maxu = float("inf")):
    """Initializes the PID controller tuning parameters such as initial position (y), the three constant gains (kp, ki, kd), the error (e), the
    total accumulated errors (i), and the rate of change of the error (dedt). Also preinitializes prev_e to 0 to prevent errors in the control
    loop, along with allowing for the calculation of dedt. Also clamps u to a specific range between minu and maxu. Also takes in dt to calculate
    dedt and i later in the control loop. Even though this is for only one dimension, this can be used for each component in a 3d vector."""
    self.y = y
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.e = 0
    self.i = 0
    self.prev_e = 0
    self.dedt = 0
    self.minu = minu
    self.maxu = maxu
    self.u = 0
    self.dt = dt
  def update(self, setpoint, y):
    """Computes and returns an input value (u) that is intended to be used. u is clamped according to minu and maxu and is calculated using the PID
    formula as shown below. Recalculates dedt and e every iteration given the setpoint and y. Pairs well with an alpha-beta (ab) or a 
    simplified Kalman filter."""
    self.y = y
    self.e = setpoint - self.y
    self.dedt = (self.e - self.prev_e) / self.dt
    self.u = self.kp * self.e + self.ki * self.i + self.kd * self.dedt
    if self.u > self.maxu:
      self.u = self.maxu
    elif self.u < self.minu:
      self.u = self.minu
    self.i += self.e * self.dt
    self.prev_e = self.e
    return self.u
