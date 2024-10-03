import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.integrate import solve_ivp

class Dynamics:
    def __init__(self, initial_state, parameters, t_span, t_eval, control_algorithm=None):
        self.initial_state = initial_state
        self.parameters = parameters
        self.t_span = t_span
        self.t_eval = t_eval
        self.control_algorithm = control_algorithm  # Function that modifies parameters during the simulation

    def complex_dynamics(self, t, state):
        x, y, u, v, psai, r = state
        gamma, angleS, angleR, Vw, P4, P5, P6, P7, P8, P9, P10 = self.parameters

        # Hydrodynamic and aerodynamic forces
        fr = P5 * u * np.sin(math.radians(angleR))  # Force on rudder
        fs = P4 * (Vw * np.sin(math.radians(psai - gamma + angleS)) - u * np.sin(math.radians(angleS)))  # Force on sail

        # Differential equations
        dx_dt = u * np.cos(math.radians(psai)) - v * np.sin(math.radians(psai))
        dy_dt = u * np.sin(math.radians(psai)) + v * np.cos(math.radians(psai))
        du_dt = (fs * np.sin(math.radians(angleS)) - fr * np.sin(math.radians(angleR))) / P9
        dv_dt = (-fs * np.cos(math.radians(angleS)) + fr * np.sin(math.radians(angleR))) / P9
        dpsai_dt = r
        dr_dt = (-P8 * fr * np.cos(math.radians(angleR)) + (P6 - P7 * np.cos(math.radians(angleS))) * fs) / P10

        return np.array([dx_dt, dy_dt, du_dt, dv_dt, dpsai_dt, dr_dt])

    def step_simulation(self, t_start, t_end, state):
        t_span = [t_start, t_end]
        solution = solve_ivp(self.complex_dynamics, t_span, state, t_eval=[t_end])
        return solution.y[:, -1]  # Return the state at the end of the step

    def solve_dynamics(self):
        states = [self.initial_state]
        current_state = self.initial_state
        times = [self.t_eval[0]]

        for i in range(1, len(self.t_eval)):
            t_start = self.t_eval[i - 1]
            t_end = self.t_eval[i]

            # Apply the control algorithm if provided
            if self.control_algorithm:
                self.parameters = self.control_algorithm(t_start, self.parameters, current_state)

            # Simulate for the current time step
            next_state = self.step_simulation(t_start, t_end, current_state)
            current_state = next_state
            states.append(next_state)
            times.append(t_end)

        return np.array(times), np.array(states)

    def plot_solution(self, times, states):
        """
        Plot the state variables over time.

        :param times: Time points at which the solution was computed
        :param states: Array of state variables over time
        """
        plt.figure(figsize=(10, 8))
        for i in range(6):
            plt.plot(times, states[:, i], label=f"x{i + 1} (t)")
        plt.xlabel('Time')
        plt.ylabel('State Variables')
        plt.legend()
        plt.title('Complex Nonlinear Dynamic System Simulation with Parameter Control')
        plt.show()

# Example control algorithm that modifies angleS and angleR
def control_algorithm(t, parameters, state):
    gamma, angleS, angleR, Vw, P4, P5, P6, P7, P8, P9, P10 = parameters

    # Adjust angles based on time or state
    if t > 5:
        angleS += 5  # Example: Increase sail angle after t = 5
        angleR -= 5  # Example: Decrease rudder angle after t = 5

    # Ensure the angles are within valid ranges
    angleS = np.clip(angleS, -30, 30)  # Example: limit sail angle between -30 and 30 degrees
    angleR = np.clip(angleR, -30, 30)  # Example: limit rudder angle between -30 and 30 degrees

    return [gamma, angleS, angleR, Vw, P4, P5, P6, P7, P8, P9, P10]

# Example usage
if __name__ == "__main__":
    initial_state = [0, 0, 0, 0, 0, 0]  # Initial state
    parameters = [1, 0, 0, 4, 5, 6, 0.5, 0.45, 0.3, 3, 1]  # Parameters
    t_span = [0, 10]  # Time span
    t_eval = np.linspace(t_span[0], t_span[1], 1000)  # Time points for evaluation

    dynamics = Dynamics(initial_state, parameters, t_span, t_eval, control_algorithm=control_algorithm)
    times, states = dynamics.solve_dynamics()  # Solve the dynamics with control
    dynamics.plot_solution(times, states)  # Plot the results
