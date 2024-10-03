import math
import optuna
from xfoil import XFoil
from xfoil.test import naca0012
import matplotlib.pyplot as plt
from Sail import Sail
import numpy as np

# Define Parameters
mass = 6  # mass of the drone in kg
Cl = 1.5  # lift coefficient of the sail
Cd = 0.017  # drag coefficient of the sail
rho = 1.5  # air density in kg/m^3
A_sail = 1  # sail area in m^2
V_wind = 10  # wind velocity in m/s
theta_wind = 20  # wind direction in degrees (relative to the drone)
alpha_sail = 0  # initial sail angle (degrees)
time_step = 0.05  # time step in seconds
total_time = 10  # total simulation time in seconds

# Initial conditions
x_pos = 0
x_vel = 0
x_acc = 0

y_pos = 0
y_vel = 0
y_acc = 0

x_positions = []
y_positions = []

# Function to calculate Cd based on AoA
def calculateClCd(AoA):
    AoA = round(AoA)
    print(f"Checking AoA: {AoA}")

    if np.absolute(AoA) >= 30:
        cl = 1
        cd = 0.1  #???????????????????????????????
        return cl,cd
    else:
        cl, cd, _, _ = xf.a(AoA)

        # Handle missing Cl values
        if math.isnan(cl):
            print(f"Cl data missing at AoA: {AoA}")
            # Search for valid cl within a range of 20 increments
            for i in range(20):
                cl_pos = xf.a(AoA + i + 1)[0]  # cl at AoA + i + 1
                cl_neg = xf.a(AoA - i - 1)[0]  # cl at AoA - i - 1
                print(f"Checking Cl at AoA + {i + 1}: {cl_pos}, AoA - {i + 1}: {cl_neg}")

                if not math.isnan(cl_pos):  # If positive AoA has valid cl
                    cl = cl_pos
                    print(f"Replaced Cl with value from AoA + {i + 1}: {cl}")
                    break
                elif not math.isnan(cl_neg):  # If negative AoA has valid cl
                    cl = cl_neg
                    print(f"Replaced Cl with value from AoA - {i + 1}: {cl}")
                    break

        # Handle missing Cd values
        if math.isnan(cd):
            print(f"Cd data missing at AoA: {AoA}")
            # Search for valid cd within a range of 20 increments
            for i in range(20):
                cd_pos = xf.a(AoA + i + 1)[1]  # cd at AoA + i + 1
                cd_neg = xf.a(AoA - i - 1)[1]  # cd at AoA - i - 1
                print(f"Checking Cd at AoA + {i + 1}: {cd_pos}, AoA - {i + 1}: {cd_neg}")

                if not math.isnan(cd_pos):  # If positive AoA has valid cd
                    cd = cd_pos
                    print(f"Replaced Cd with value from AoA + {i + 1}: {cd}")
                    break
                elif not math.isnan(cd_neg):  # If negative AoA has valid cd
                    cd = cd_neg
                    print(f"Replaced Cd with value from AoA - {i + 1}: {cd}")
                    break

        return cl, cd

# Function to calculate force in the x direction
def calculateFx(Cl, Cd, theta_wind):
    F_D = 0.5 * Cd * rho * A_sail * V_wind ** 2
    F_L = 0.5 * Cl * rho * A_sail * V_wind ** 2
    F_x = F_L * math.sin(math.radians(theta_wind)) - F_D * math.cos(math.radians(theta_wind))
    return F_x


# Function to calculate force in the y direction
def calculateFy(Cl, Cd, theta_wind):
    F_D = 0.5 * Cd * rho * A_sail * V_wind ** 2
    F_L = 0.5 * Cl * rho * A_sail * V_wind ** 2
    F_y = F_L * math.cos(math.radians(theta_wind)) + F_D * math.sin(math.radians(theta_wind))
    return F_y

# alpha is the windvane angle,[0,360); beta is the airfoil angle, [0, 180]
def convertAngle(alpha,beta):
    beta -= 90 #beta [-90,90]
    if 180<alpha and alpha<360:
        alpha -= 360 #alpah (-180,180]
    theta = alpha - beta #theta (-270,270]
    if theta > -270 and theta < -180:
        theta += 360
    elif theta > 180 and theta < 270:
        theta -= 360

    if theta <= 180 and theta >= -180:
        return theta
    else:
        print("error!")

def setWindDirection(t):
    if t < 5:
        angle = 20
    else:
        angle = -30
    return angle

if __name__ == '__main__':
    sail = Sail()
    xf = XFoil()
    xf.airfoil = naca0012
    xf.Re = 1e6
    xf.mat_iter = 40
    # Time loop
    for t in range(0, int(total_time / time_step)):
        # Convert t to real time
        t = t * time_step

        # Calculate relative wind angle in radians
        sail.moveSailTo(theta_wind)
        alpha_sail = sail.currentSailPos
        # alpha_sail -= 90

        # if 0<=alpha_sail and alpha_sail<90:
        #     alpha_sail+=270
        # elif 90<=alpha_sail and alpha_sail<=180:
        #     alpha_sail-=90
        # print("!!!!!!!!!!!!!!!!!!",alpha_sail)
        # # theta = math.radians(theta_wind - alpha_sail)
        # theta = theta_wind - alpha_sail
        print("Checking alpha sail", alpha_sail)
        theta_wind = setWindDirection(t)
        theta = convertAngle(alpha_sail,theta_wind)

        Cl, Cd = calculateClCd(theta)

        F_x = calculateFx(Cl, Cd, theta_wind)
        F_y = calculateFy(Cl, Cd, theta_wind)

        # Acceleration in x and y directions
        x_acc = F_x / mass
        y_acc = F_y / mass

        # Update velocity and position
        x_vel += x_acc * time_step
        x_pos += x_vel * time_step
        y_vel += y_acc * time_step
        y_pos += y_vel * time_step

        if math.isnan(x_pos) or math.isnan(y_pos) or math.isnan(x_vel) or math.isnan(y_vel):
            print("!!!!!!!!!!!!!!!!!!!!!!!!",t)
            print("!!!",alpha_sail,"!!!",theta)
            break

        x_positions.append(x_pos)
        y_positions.append(y_pos)

        # Display or store results
        print(f"X::Time: {t:.2f} s, Position: {x_pos:.2f} m, Velocity: {x_vel:.2f} m/s, Acceleration: {x_acc:.2f} m/s^2")
        print(f"Y::Time: {t:.2f} s, Position: {y_pos:.2f} m, Velocity: {y_vel:.2f} m/s, Acceleration: {y_acc:.2f} m/s^2")

        # Plotting the motion track
    plt.figure(figsize=(10, 6))
    plt.plot(x_positions, y_positions, marker='o', linestyle='-', color='b')
    plt.title('Drone Motion Track')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid()
    plt.axis('equal')
    plt.show()