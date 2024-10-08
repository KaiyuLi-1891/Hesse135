import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.integrate import solve_ivp
from Rudder import Rudder
from Sail import Sail

class Dynamics:
    def __init__(self,
                 initial_state = [0,0,0,0,0,0], #Initial state of the system [x, y, u, v, psai, omega]
                 parameters = [1, 0, 0, 4, 0, 0 ,0, 5, 6, 0.5, 0.45, 0.3, 3, 1, 0, 0, 0],
                 #System parameters [gamma, angleS, angleR, Vw, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, amU, amV, amOmega]
                 t_span = [0,10], #Time span for the simulation [start_time, end_time]
                 t_eval_index = 1000, #Time points at which to store the computed solutions
                 control_algorithm = None,
                 targetX = 0,
                 targetY = 0
    ):

        self.initial_state = initial_state
        self.parameters = parameters
        self.t_span = t_span
        self.t_eval = np.linspace(self.t_span[0],self.t_span[1],t_eval_index)
        self.control_algorithm = control_algorithm
        self.targetX = targetX
        self.targetY = targetY

        self.current_state = self.initial_state

        self.x = self.initial_state[0]
        self.y = self.initial_state[1]
        self.u = self.initial_state[2]
        self.v = self.initial_state[3]
        self.psai = self.initial_state[4]
        self.omega = self.initial_state[5]

        self.gamma = self.parameters[0]
        self.angleS = self.parameters[1]
        self.angleR = self.parameters[2]
        self.Vw = self.parameters[3]

        self.rudder = Rudder()
        self.sail = Sail()


    def complex_dynamics(self, t, state):
        x, y, u, v, psai, omega = state
        gamma, angleS, angleR, Vw, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, amU, amV, amOmega = self.parameters

        # Hydrodynamic and aerodynamic forces
        fr = P5 * u * np.sin(math.radians(angleR))  # Force on rudder
        fs = P4 * (Vw * np.sin(math.radians(psai - gamma + angleS)) - u * np.sin(math.radians(angleS)))  # Force on sail

        # Differential equations
        dx_dt = u * np.cos(math.radians(psai)) - v * np.sin(math.radians(psai))
        dy_dt = u * np.sin(math.radians(psai)) + v * np.cos(math.radians(psai))
        du_dt = (fs * np.sin(math.radians(angleS)) - fr * np.sin(math.radians(angleR)) - P1 * u) / (P9 - amU)
        dv_dt = (-fs * np.cos(math.radians(angleS)) + fr * np.sin(math.radians(angleR)) - P2 * v) / (P9 - amV)
        dpsai_dt = omega
        domega_dt = (-P8 * fr * np.cos(math.radians(angleR)) + (P6 - P7 * np.cos(math.radians(angleS))) * fs - P3 * omega) / (P10 - amOmega)

        return np.array([dx_dt, dy_dt, du_dt, dv_dt, dpsai_dt, domega_dt])

    def step_simulation(self, t_start, t_end, state):
        t_span = [t_start, t_end]
        solution = solve_ivp(self.complex_dynamics,t_span,state,t_eval = [t_end],method = 'RK45')
        return solution.y[:,-1]

    def solve_dynamics(self):
        states = [self.initial_state]
        self.current_state = self.initial_state
        times = [self.t_eval[0]]

        for i in range(1, len(self.t_eval)):
            t_start = self.t_eval[i - 1]
            t_end = self.t_eval[i]

            # if self.control_algorithm:
            #     self.parameters = self.control_algorithm(t_start, self.parameters, self.current_state)
            self.control_algorithm1(t_start)

            print("t:", t_start, "S:", self.angleS, "R:", self.angleR, "psai:", self.psai, "u:", self.u, "v:", self.v)

            next_state = self.step_simulation(t_start, t_end, self.current_state)
            self.current_state = next_state
            self.x = self.current_state[0]
            self.y = self.current_state[1]
            self.u = self.current_state[2]
            self.v = self.current_state[3]
            self.psai = self.current_state[4]
            self.omega = self.current_state[5]
            states.append(next_state)
            times.append(t_end)

        return np.array(times), np.array(states)

    def control_algorithm1(self,t_start):
        self.angleS = self.sail.sailControl(self.psai, self.gamma, self.angleS)
        self.angleR = self.rudder.rudderControl(self.current_state[0],self.current_state[1],self.current_state[4],self.targetX,self.targetY)
        self.parameters[1] = self.angleS
        self.parameters[2] = self.angleR
        # print("t:",t_start,"S:", self.angleS,"R:",self.angleR,"psai:",self.psai,"u:",self.u,"v:",self.v)
    def plot_solution(self, times, states):
        plt.figure(figsize=(10, 8))
        for i in range(6):
            # plt.plot(times, states[:, i], label=f"x{i + 1} (t)")
            plt.plot(states[:,0],states[:,1])
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.title('Complex Nonlinear Dynamic System Simulation with Parameter Control')
        plt.show()


if __name__ == "__main__":
    initial_state = [0, 0, 0, 0, 0, 0]
    parameters = [0, 90, 0, 5, 0, 0, 0, 0.596, 45.98, 0, 0.38, 0.3, 7.18, 2, 0, 0, 0]
    t_span = [0, 20]
    # t_eval = np.linspace(t_span[0], t_span[1], 1000)
    t_eval_index = 1000

    dynamics = Dynamics(initial_state, parameters, t_span, t_eval_index)
    times, states = dynamics.solve_dynamics()  # Solve the dynamics
    dynamics.plot_solution(times,states)  # Plot the results



