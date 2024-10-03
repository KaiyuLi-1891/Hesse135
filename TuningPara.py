import optuna
import numpy as np

def simulate_system(controller, t_simulation, wind_speed):
    measured_value = 0
    error_accumulated = 0

    for t in np.arange(0, t_simulation, controller.dt):
        control_signal = controller.control(measured_value)
        measured_value += control_signal - wind_speed
        error = abs(controller.setpoint - measured_value)
        error_accumulated += error * controller.dt

    return error_accumulated


def objective(trial):
    Kp = trial.suggest_float('Kp', 0.0, 10.0)
    Ki = trial.suggest_float('Ki', 0.0, 10.0)
    Kd = trial.suggest_float('Kd', 0.0, 10.0)

    # Simulation settings
    setpoint = 1.0  # Desired sail position
    dt = 0.01  # Time step (in seconds)
    t_simulation = 10.0  # Total simulation time
    wind_speed = 0.2  # Constant wind speed

    # Create a PID controller instance with suggested parameters
    pid = PIDController(Kp, Ki, Kd, setpoint, dt)

    # Run simulation and get performance index (IAE in this case)
    performance_index = simulate_system(pid, t_simulation, wind_speed)

    return performance_index

# Create and optimize the study
study = optuna.create_study(direction='minimize')
study.optimize(objective, n_trials=100)

# Print the best parameters found
print(f"Best PID parameters: {study.best_params}")
print(f"Best performance index: {study.best_value}")

