import numpy as np
import matplotlib.pyplot as plt

class Wind:
    def __init__(self, meanV=5, varV=2, meanGamma=45, varGamma=0.5, tau=10, dt=0.1):
        # meanV: mean value of velocity; varV: variance of velocity;
        # meanGamma: mean value of wind direction; varGamma: variance of wind direction.
        # tau: the time constant to smooth the change of the wind direction and velocity
        self.meanV = meanV
        self.varV = varV
        self.meanGamma = meanGamma * np.pi / 180
        self.varGamma = varGamma
        self.tau = tau
        self.dt = dt

        # Calculate Weibull scale (lambda) and shape (k) parameters from mean and variance
        self.kV = (self.varV / (self.meanV ** 2)) ** (-1 / 2)  # Shape parameter (k)
        self.lambdaV = self.meanV / np.sqrt(np.pi / 2)  # Scale parameter (lambda)

    def getWind(self):
        # Weibull distribution for the wind velocity.
        self.V = self.meanV
        self.Gamma = self.meanGamma * 180 / np.pi

        new_V = np.random.weibull(self.kV) * self.lambdaV
        self.V += (new_V - self.V) * self.dt / self.tau

        # Von Mises distribution for the wind direction.
        new_Gamma = np.random.vonmises(self.meanGamma, 1 / self.varGamma)  # Concentration = 1 / variance
        new_Gamma = new_Gamma * 180 / np.pi
        self.Gamma += (new_Gamma - self.Gamma) * self.dt / self.tau

        return self.V, self.Gamma

if __name__ == "__main__":
    mean_velocity = 10  
    variance_velocity = 5  
    mean_direction = 45   
    variance_direction = 0.5

    index_list = []
    velocity_list = []
    direction_list = []

    wind = Wind(mean_velocity, variance_velocity, mean_direction, variance_direction)
    for i in range(500):
        index_list.append(i)
        V, Gamma = wind.getWind()
        velocity_list.append(V)
        direction_list.append(Gamma)

    plt.plot(index_list, velocity_list)
    plt.show()
    plt.plot(index_list, direction_list)
    plt.show()


