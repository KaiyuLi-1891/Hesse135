import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
from scipy.integrate import solve_ivp
from Rudder import Rudder
from Sail import Sail
from Wind import Wind

class Dynamics:
    def __init__(self,
                 initial_state = [0, 0, 0, 0, 0, 0, 0, 0], #Initial state of the system [x, y, u, v, psi, omega, angleS, angleR]
                 parameters = [60, 4, 1, 20 ,0.6, 5, 6, 0.5, 0.45, 0.3, 14.12, 10, 0.05*3, 0.2*3, 0.36*1],
                 #System parameters [gamma, Vw, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, amU, amV, amOmega]
                 t_span = [0,10], #Time span for the simulation [start_time, end_time]
                 t_eval_index = 1000, #Time points at which to store the computed solutions
                 control_algorithm = None,
                 targetX = 20,
                 targetY = 10,
                 force_order = 1, # if 1, the force is proportional to u; if 2, the force is proportional to u^2
                 Kp = 0.8,
                 Ki = 0,
                 Kd = 0,
                 meanV = 5,
                 varV = 2,
                 meanGamma = -30,
                 varGamma = 0.5,
                 tau = 0.5

    ):

        self.initial_state = initial_state
        self.parameters = parameters
        self.t_span = t_span
        self.t_eval_index = t_eval_index
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
        self.sail = Sail(Kp=Kp,Ki=Ki,Kd=Kd)
        self.wind = Wind(meanV=meanV, varV=varV, meanGamma=meanGamma, varGamma=varGamma, tau=tau, dt=(self.t_span[1]-self.t_span[0])/self.t_eval_index)

        self.bearingErrorList = []
        initial_error = self.calculateBearingError()
        self.bearingErrorList.append(initial_error)

        self.windVelocityList = [meanV]
        self.windDirectionList = [meanGamma]
        self.control_input1_list = [0] # Record the control input for the sail
        self.control_input2_list = [0] # Record the control input for the rudder


    def complex_dynamics(self, t, state):
        x, y, u, v, psi, omega, angleS, angleR = state
        gamma, Vw, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, amU, amV, amOmega = self.parameters
        gamma = self.gamma
        Vw = self.Vw

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

            self.Vw, self.gamma = self.wind.getWind()
            self.windVelocityList.append(self.Vw)
            self.windDirectionList.append(self.gamma)

            if self.control_algorithm:
                self.control_input = self.control_algorithm(t_start, self.parameters, self.current_state)
            else:
                self.control_algorithm1(t_start)


            next_state = self.step_simulation(t_start, t_end, self.current_state)
            self.current_state = next_state
            self.x = self.current_state[0]
            self.y = self.current_state[1]
            self.u = self.current_state[2]
            self.v = self.current_state[3]
            self.current_state[4] = np.mod(self.current_state[4],360)
            self.psi = self.current_state[4]
            self.current_state[5] = np.mod(self.current_state[5], 360)
            self.omega = self.current_state[5]
            self.current_state[6] = np.mod(self.current_state[6], 360)
            self.angleS = self.current_state[6]
            self.current_state[7] = np.mod(self.current_state[7], 360)
            self.angleR = self.current_state[7]


            states.append(self.current_state)
            times.append(t_end)

            print("t:", t_start, "S:", self.angleS, "R:", self.angleR, "psi:", self.psi, "u:", self.u, "v:", self.v)

            error = self.calculateBearingError()
            self.bearingErrorList.append(error)

        return np.array(times), np.array(states)

    def control_algorithm1(self,t_start):
        # self.angleS = self.sail.sailControl(self.psi, self.gamma, self.angleS)
        # self.angleR = self.rudder.rudderControl(self.current_state[0],self.current_state[1],self.current_state[4],self.targetX,self.targetY)
        self.control_input[0] = self.sail.sailControl(self.psi, self.gamma, self.angleS)
        self.control_input[1] = self.rudder.rudderControl(self.current_state[0],self.current_state[1],self.current_state[4],self.targetX,self.targetY)
        self.control_input1_list.append(self.control_input[0])
        self.control_input2_list.append(self.control_input[1])
        # self.control_input[0] = np.clip(self.control_input[0],-10,10)
        # print("t:",t_start,"S:", self.angleS,"R:",self.angleR,"psi:",self.psi,"u:",self.u,"v:",self.v)
    def plot_solution(self, times, states):
        fig, axs = plt.subplots(3, 3, figsize=(10, 8))

        axs[0, 0].plot(states[:, 0], states[:, 1], color='blue')
        axs[0, 0].scatter(self.targetX, self.targetY, color='red', s=40)
        axs[0, 0].set_title('Sailboat Trajectory')
        axs[0, 0].set_xlabel('X (m)')
        axs[0, 0].set_ylabel('Y (m)')

        axs[0, 1].plot(times, states[:, 4], color='blue')
        axs[0, 1].set_title('Sailboat Bearing')
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylabel('Bearing ')

        axs[0, 2].plot(times, self.bearingErrorList, color='red')
        axs[0, 2].set_title('Bearing Error')
        axs[0, 2].set_xlabel('Time (s)')
        axs[0, 2].set_ylabel('Bearing Error(degrees)')

        axs[1, 0].plot(times, self.windDirectionList, color='green')
        axs[1, 0].set_title('Wind Direction')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('Wind Direction (degrees)')

        axs[2, 0].plot(times, self.windVelocityList, color='green')
        axs[2, 0].set_title('Wind Velocity')
        axs[2, 0].set_xlabel('Time (s)')
        axs[2, 0].set_ylabel('Wind Velocity (m/s)')

        axs[2, 1].plot(times, states[:,6], color= 'blue')
        axs[2, 1].set_title('Sail Angle (angleS)')
        axs[2, 1].set_xlabel('Time (s)')
        axs[2, 1].set_ylabel('Sail Angle (degrees)')

        axs[2, 2].plot(times, states[:, 7], color = 'blue')
        axs[2, 2].set_title('Rudder Angle (angleR)')
        axs[2, 2].set_xlabel('Time (s)')
        axs[2, 2].set_ylabel('Rudder Angle (degrees)')

        axs[1, 1].plot(times, self.control_input1_list, color='orange')
        axs[1, 1].set_title('Sail Control Input')
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylabel('Sail Control Input(degrees)')

        axs[1, 2].plot(times, self.control_input2_list, color='orange')
        axs[1, 2].set_title('Rudder Control Input')
        axs[1, 2].set_xlabel('Time (s)')
        axs[1, 2].set_ylabel('Rudder Control Input(degrees)')

        plt.tight_layout()
        plt.show()

    def generate_gif(self, times, states, filename='sailboat_simulation.gif'):
        fig, axs = plt.subplots(3, 3, figsize=(10, 8))

        line_trajectory, = axs[0, 0].plot([], [], color='blue')
        scatter_target = axs[0, 0].scatter(self.targetX, self.targetY, color='red', s=40)
        axs[0, 0].set_title('Sailboat Trajectory')
        axs[0, 0].set_xlabel('X (m)')
        axs[0, 0].set_ylabel('Y (m)')

        line_bearing, = axs[0, 1].plot([], [], color='blue')
        axs[0, 1].set_title('Sailboat Bearing')
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylabel('Bearing')

        line_bearing_error, = axs[0, 2].plot([], [], color='red')
        axs[0, 2].set_title('Bearing Error')
        axs[0, 2].set_xlabel('Time (s)')
        axs[0, 2].set_ylabel('Bearing Error (degrees)')

        line_wind_dir, = axs[1, 0].plot([], [], color='green')
        axs[1, 0].set_title('Wind Direction')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('Wind Direction (degrees)')

        line_wind_vel, = axs[2, 0].plot([], [], color='green')
        axs[2, 0].set_title('Wind Velocity')
        axs[2, 0].set_xlabel('Time (s)')
        axs[2, 0].set_ylabel('Wind Velocity (m/s)')

        line_sail_angle, = axs[2, 1].plot([], [], color='blue')
        axs[2, 1].set_title('Sail Angle (angleS)')
        axs[2, 1].set_xlabel('Time (s)')
        axs[2, 1].set_ylabel('Sail Angle (degrees)')

        line_rudder_angle, = axs[2, 2].plot([], [], color='blue')
        axs[2, 2].set_title('Rudder Angle (angleR)')
        axs[2, 2].set_xlabel('Time (s)')
        axs[2, 2].set_ylabel('Rudder Angle (degrees)')

        line_sail_control, = axs[1, 1].plot([], [], color='orange')
        axs[1, 1].set_title('Sail Control Input')
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylabel('Sail Control Input (degrees)')

        line_rudder_control, = axs[1, 2].plot([], [], color='orange')
        axs[1, 2].set_title('Rudder Control Input')
        axs[1, 2].set_xlabel('Time (s)')
        axs[1, 2].set_ylabel('Rudder Control Input (degrees)')

        plt.tight_layout()

        # Define animation function
        def animate(i):
            # Update each plot dynamically
            line_trajectory.set_data(states[:i, 0], states[:i, 1])
            line_bearing.set_data(times[:i], states[:i, 4])
            line_bearing_error.set_data(times[:i], self.bearingErrorList[:i])
            line_wind_dir.set_data(times[:i], self.windDirectionList[:i])
            line_wind_vel.set_data(times[:i], self.windVelocityList[:i])
            line_sail_angle.set_data(times[:i], states[:i, 6])
            line_rudder_angle.set_data(times[:i], states[:i, 7])
            line_sail_control.set_data(times[:i], self.control_input1_list[:i])
            line_rudder_control.set_data(times[:i], self.control_input2_list[:i])
            return (line_trajectory, line_bearing, line_bearing_error, line_wind_dir,
                    line_wind_vel, line_sail_angle, line_rudder_angle, line_sail_control,
                    line_rudder_control)

        # Create animation
        anim = FuncAnimation(fig, animate, frames=len(times), interval=100, blit=True)

        # Save animation as GIF
        anim.save(filename, writer='pillow')

        plt.show()
        print("finished")

    def calculateBearingError(self):
        targetBearing = math.atan2(self.targetY-self.y, self.targetX-self.x)
        targetBearing = math.degrees(targetBearing)

        while targetBearing < 360:
            targetBearing += 360

        currentBearing = self.psi

        if currentBearing - targetBearing <= 180 and currentBearing - targetBearing >= -180:
            error = np.abs(currentBearing - targetBearing)
        elif currentBearing - targetBearing > 180:
            error = 360 - (currentBearing - targetBearing)
        elif currentBearing - targetBearing < -180:
            error = 360 + currentBearing - targetBearing

        return error




if __name__ == "__main__":
    # initial_state = [0, 0, 0, 0, 0, 0]
    # parameters = [0, 90, 0, 5, 0, 0, 0, 0.596, 45.98, 0, 0.38, 0.3, 7.18, 2, 0, 0, 0]
    # t_span = [0, 20]
    # # t_eval = np.linspace(t_span[0], t_span[1], 1000)
    # t_eval_index = 1000

    dynamics = Dynamics()
    times, states = dynamics.solve_dynamics()
    dynamics.plot_solution(times,states)
    # dynamics.generate_gif(times,states)


