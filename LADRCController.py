class LADRCController:
  """ This class is an implementation of LADRC, an error correction method. Designed in 2003 by Dr. Zhiqiang Gao,
  LADRC is a closed-loop system that actively cancels out the external forces that result in error while being intuitive, improving on
  Dr. Jingqing Han's work on conventional ADRC. Gao's algorithm has a built in LESO to accomodate for sensor noise. 
  Also, this python implementation features two parameters, minu and maxu, which can help users of this accommodate for their plant's
  capabilities. The only tuning parameters that are required are wc (bandwidth of the controller), wo (bandwith of the observer), b0, and dt.
  This can only be applicable for one dimension, but can be utilized in 3d via 3d vectors.
  """
  def __init__(self, wc, wo, b0, dt, z1 = 0, z2 = 0, z3 = 0, minu = -float("inf"), maxu = float("inf")):
    """Initializes tuning parameters defined above. Defines beta1, beta2, beta3, kp, kd using the formulas according to Gao's work. Parameters of this
    function include, wc, wo, b0, dt, z1, z2, z3, minu, and maxu. wc and wo are the frequencies of the controller and observer, respectively, while b0
    is a factor that translates the input (u) into an acceleration magnitude, depending on the plant's design. minu and maxu are the minimum and maximum signals that can be sent to the plant, respectively.
    z1, z2, and z3 are estimates of the plant's position, velocity, and acceleration. dt is simply the time step of the controller. """
    self.beta1 = 3 * wo
    self.beta2 = 3 * wo * wo
    self.beta3 = wo * wo * wo
    self.kp = wc * wc
    self.kd = 2 * wc
    self.y = 0
    self.b0 = b0
    self.dt = dt
    self.z1 = z1
    self.z2 = z2
    self.z3 = z3
    self.u = 0
    self.minu = minu
    self.maxu = maxu
  def update(self, setpoint, y):
    """Uses a LESO to accomodate for sensor noise and estimate the plant's position. Based on this, an error will be estimated using the given setpoint and
    the estimated position. u is capped between minu and maxu for optimal accuracy. Returns u.
    """
    self.y = y
    self.setpoint = setpoint
    self.obs_error = self.z1 - self.y
    self.z1 += self.dt * (self.z2 - self.beta1 * self.obs_error)
    self.z2 += self.dt * (self.z3 - self.beta2 * self.obs_error + self.b0 * self.u)
    self.z3 += self.dt * -self.beta3 * self.obs_error
    self.set_error = self.setpoint - self.z1
    self.u0 = self.kp * self.set_error - self.kd * self.z2
    self.u = (self.u0 - self.z3)/self.b0
    if self.u < self.minu:
      self.u = self.minu
    elif self.u > self.maxu:
      self.u = self.maxu
    return self.u
