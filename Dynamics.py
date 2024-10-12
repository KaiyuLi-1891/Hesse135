import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.integrate import solve_ivp
from Rudder import Rudder
from Sail import Sail

class Dynamics:
    def __init__(self,
                 initial_state = [0, 0, 0, 0, 0, 0, 0, 0], #Initial state of the system [x, y, u, v, psi, omega, angleS, angleR]
                 parameters = [1, 4, 1, 5 ,0.6, 5, 6, 0.5, 0.45, 0.3, 6, 1, 0.05*3, 0.2*3, 0.36*1],
                 #System parameters [gamma, Vw, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, amU, amV, amOmega]
                 t_span = [0,20], #Time span for the simulation [start_time, end_time]
                 t_eval_index = 1000, #Time points at which to store the computed solutions
                 control_algorithm = None,
                 targetX = 0,
                 targetY = 0,
                 force_order = 2 # if 1, the force is proportional to u; if 2, the force is proportional to u^2
    ):

        self.initial_state = initial_state
        self.parameters = parameters
        self.t_span = t_span
        self.t_eval = np.linspace(self.t_span[0],self.t_span[1],t_eval_index)
        self.control_algorithm = control_algorithm
        self.targetX = targetX
        self.targetY = targetY
        self.force_order = force_order

        self.current_state = self.initial_state
        self.control_input = [0,0]

        self.x = self.initial_state[0]
        self.y = self.initial_state[1]
        self.u = self.initial_state[2]
        self.v = self.initial_state[3]
        self.psi = self.initial_state[4]
        self.omega = self.initial_state[5]
        self.angleS = self.initial_state[6]
        self.angleR = self.initial_state[7]

        self.gamma = self.parameters[0]
        self.Vw = self.parameters[1]

        self.rudder = Rudder()
        self.sail = Sail()


    def complex_dynamics(self, t, state):
        x, y, u, v, psi, omega, angleS, angleR = state
        gamma, Vw, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, amU, amV, amOmega = self.parameters

        # Hydrodynamic and aerodynamic forces
        if self.force_order == 1:
            fr = P5 * u**2 * np.sin(math.radians(angleR))  # Force on rudder
            fs = P4 * np.sin(math.radians(psi - gamma + angleS)) * np.sqrt(((Vw*np.sin(math.radians(gamma-psi))-v)**2 + (Vw*np.cos(math.radians(gamma-psi))-u)**2))  # Force on sail
        elif self.force_order == 2:
            fr = P5 * u **2 * np.sin(math.radians(angleR))  # Force on rudder
            fs = P4 * np.sin(math.radians(psi - gamma + angleS)) * ((Vw * np.sin(math.radians(gamma - psi)) - v) ** 2 + (Vw * np.cos(math.radians(gamma - psi)) - u) ** 2)
        # Differential equations
        dx_dt = u * np.cos(math.radians(psi)) - v * np.sin(math.radians(psi))
        dy_dt = u * np.sin(math.radians(psi)) + v * np.cos(math.radians(psi))
        du_dt = (fs * np.sin(math.radians(angleS)) - fr * np.sin(math.radians(angleR)) - P1 * u) / (P9 - amU)
        dv_dt = (-fs * np.cos(math.radians(angleS)) + fr * np.sin(math.radians(angleR)) - P2 * v) / (P9 - amV)
        dpsi_dt = omega
        domega_dt = (-P8 * fr * np.cos(math.radians(angleR)) + (P6 - P7 * np.cos(math.radians(angleS))) * fs - P3 * omega) / (P10 - amOmega)
        dangleS_dt = self.control_input[0]
        dangleR_dt = self.control_input[1]
        return np.array([dx_dt, dy_dt, du_dt, dv_dt, dpsi_dt, domega_dt,dangleS_dt,dangleR_dt])

    def step_simulation(self, t_start, t_end, state):
        t_span = [t_start, t_end]
        solution = solve_ivp(self.complex_dynamics,t_span,state,t_eval = [t_end],method = 'RK45')

        if solution.success:
            return solution.y[:, -1]
        else:
            print("Integration failed:", solution.message)
            return state  # Return the last valid state if integration fails


    def solve_dynamics(self):
        states = [self.initial_state]
        self.current_state = self.initial_state
        times = [self.t_eval[0]]

        for i in range(1, len(self.t_eval)):
            t_start = self.t_eval[i - 1]
            t_end = self.t_eval[i]

            if self.control_algorithm:
                self.control_input = self.control_algorithm(t_start, self.parameters, self.current_state)
            else:
                # self.control_algorithm1(t_start)
                self.control_algorithm1(t_start)

            print("t:", t_start, "S:", self.angleS, "R:", self.angleR, "psi:", self.psi, "u:", self.u, "v:", self.v)
            # state = np.concatenate((self.current_state, self.control_input))
            # print("state:",state)

            next_state = self.step_simulation(t_start, t_end, self.current_state)
            self.current_state = next_state
            self.x = self.current_state[0]
            self.y = self.current_state[1]
            self.u = self.current_state[2]
            self.v = self.current_state[3]
            self.psi = self.current_state[4]
            self.omega = self.current_state[5]
            self.angleS = self.current_state[6]
            self.angleR = self.current_state[7]
            states.append(self.current_state)
            times.append(t_end)

        return np.array(times), np.array(states)

    def control_algorithm1(self,t_start):
        # self.angleS = self.sail.sailControl(self.psi, self.gamma, self.angleS)
        # self.angleR = self.rudder.rudderControl(self.current_state[0],self.current_state[1],self.current_state[4],self.targetX,self.targetY)
        self.control_input[0] = self.sail.sailControl(self.psi, self.gamma, self.angleS)
        self.control_input[1] = self.rudder.rudderControl(self.current_state[0],self.current_state[1],self.current_state[4],self.targetX,self.targetY)
        # print("t:",t_start,"S:", self.angleS,"R:",self.angleR,"psi:",self.psi,"u:",self.u,"v:",self.v)
    def plot_solution(self, times, states):
        plt.figure(figsize=(10, 8))
        for i in range(6):
            # plt.plot(times, states[:, i], label=f"x{i + 1} (t)")
            plt.plot(states[:,0],states[:,1])
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.title('The Trajectory of the Sailboat')
        plt.show()


if __name__ == "__main__":
    # initial_state = [0, 0, 0, 0, 0, 0]
    # parameters = [0, 90, 0, 5, 0, 0, 0, 0.596, 45.98, 0, 0.38, 0.3, 7.18, 2, 0, 0, 0]
    # t_span = [0, 20]
    # # t_eval = np.linspace(t_span[0], t_span[1], 1000)
    # t_eval_index = 1000

    dynamics = Dynamics()
    times, states = dynamics.solve_dynamics()  # Solve the dynamics
    dynamics.plot_solution(times,states)  # Plot the results



