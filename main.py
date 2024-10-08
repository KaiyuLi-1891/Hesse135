from Rudder import Rudder
from Sail import Sail
from Dynamics import Dynamics
import numpy as np

def control_algorithm(t, parameters, state):
    gamma, angleS, angleR, Vw, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, amU, amV, amOmega = parameters

    # Adjust angles based on time or state
    if t > 5:
        angleS += 5  # Example: Increase sail angle after t = 5
        angleR -= 5  # Example: Decrease rudder angle after t = 5

    # Ensure the angles are within valid ranges
    angleS = np.clip(angleS, -30, 30)  # Example: limit sail angle between -30 and 30 degrees
    angleR = np.clip(angleR, -30, 30)  # Example: limit rudder angle between -30 and 30 degrees

    return [gamma, angleS, angleR, Vw, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, amU, amV, amOmega]


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    rudder = Rudder()
    sail = Sail()
    dynamics = Dynamics()
    initial_state = [0, 0, 0, 0, 0, 0]
    parameters = [0, 90, 0, 5, 0, 0, 0, 0.596, 45.98, 0, 0.38, 0.3, 7.18, 2, 0, 0, 0]
    t_span = [0, 10]
    t_eval_index = 1000

    dynamics = Dynamics(initial_state, parameters, t_span, t_eval_index, control_algorithm=control_algorithm)
    times, states = dynamics.solve_dynamics()  # Solve the dynamics with control
    dynamics.plot_solution(times, states)  # Plot the results


# See PyCharm help at https://www.jetbrains.com/help/pycharm/
