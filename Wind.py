import numpy as np
import matplotlib.pyplot as plt

class Wind:
    def __init__(self, meanV=10, varV=5, meanGamma=45, varGamma=0.5):
        # meanV: mean value of velocity; varV: variance of velocity;
        # meanGamma: mean value of wind direction; varGamma: variance of wind direction.
        self.meanV = meanV
        self.varV = varV
        self.meanGamma = meanGamma * np.pi / 180
        self.varGamma = varGamma

        # Calculate Weibull scale (lambda) and shape (k) parameters from mean and variance
        self.kV = (self.varV / (self.meanV ** 2)) ** (-1 / 2)  # Shape parameter (k)
        self.lambdaV = self.meanV / np.sqrt(np.pi / 2)  # Scale parameter (lambda)

    def getWind(self):
        # Weibull distribution for the wind velocity.
        V = np.random.weibull(self.kV) * self.lambdaV

        # Von Mises distribution for the wind direction.
        Gamma = np.random.vonmises(self.meanGamma, 1 / self.varGamma)  # Concentration = 1 / variance
        Gamma = Gamma * 180 / np.pi

        # Return the velocity (V) and direction (Gamma)
        return V, Gamma

# Example usage

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


