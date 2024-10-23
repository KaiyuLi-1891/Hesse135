import optuna
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from Sail import Sail
from Dynamics import Dynamics


# PID Controller Class
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def control(self, error, dt):
        # Calculate PID terms
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output


# Objective function for Optuna
def objective(trial):
    Kp = trial.suggest_uniform('Kp', 0.0, 10.0)
    Ki = trial.suggest_uniform('Ki', 0.0, 10.0)
    Kd = trial.suggest_uniform('Kd', 0.0, 10.0)

    dynamics = Dynamics(targetX=20,targetY=10,Kp=Kp,Ki=Ki,Kd=Kd)
    times, states = dynamics.solve_dynamics()

    # Objective: Minimize the sum of squared errors
    error_sum = np.mean(dynamics.bearingErrorList)
    return error_sum

if __name__ == '__main__':
    study = optuna.create_study(direction='minimize')
    study.optimize(objective, n_trials=1000)


    print('Best parameters:', study.best_params)

    Kp_opt = study.best_params['Kp']
    Ki_opt = study.best_params['Ki']
    Kd_opt = study.best_params['Kd']

    dynamics1 = Dynamics(Kp=Kp_opt,Ki=Ki_opt,Kd=Kd_opt)
    times, states = dynamics1.solve_dynamics()
    dynamics1.plot_solution(times,states)

