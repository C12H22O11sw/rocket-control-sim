from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
from scipy.constants import mile, hour
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from parse_data import generate_bla, func_from_csv

def generate_drag_function(area: float, cd):
    drag = lambda state : 0.5 * cd(state) * area * state.density * state.vel**2
    return drag

def generate_dynamics(area, mass, cd_func, density_func, control_func=None, control_plant=None):
    accel = 0
    def system(t, state):
        pos, vel = state
        nonlocal accel
        cd = cd_func(vel)
        density = density_func(pos)
        drag = 0.5 * density * cd * area * vel**2
        net_force = -drag
        if control_func is not None and control_plant is not None:
            signal = pos, vel, accel
            k = control_func(t, signal)
            net_force += control_plant(k, vel)
        accel = net_force / mass - 9.81
        return vel, accel

    return system

def generate_control_plant_prop(filename, num_props=1):
    bla = generate_bla(filename, 'Thrust')
    def plant(rpm, vel):
        force = num_props * bla([rpm, vel * (hour / mile)])
        force = force[0]
        return force
    return plant


def main():
    (min_vel, max_vel), cd_func = func_from_csv('data/cd_function.csv', 'Airspeed', 'CD')
    (min_density, max_density), density_func = func_from_csv('data/international_standard_atmosphere.csv', 'Altitude', 'Density')
    def control_func(t, signal):
        pos, vel, accel = signal
        k = min(30000 * accel, 30000)
        return k
    
    control_plant = generate_control_plant_prop('data/PER3_6x6E.dat', 4)

    area = np.pi/4 * (5.63e-2)**2
    mass = 268 * 1e-3

    system = generate_dynamics(area, mass, cd_func, density_func, control_func, control_plant)

    state0 = [0, 90]
    
    t0 = 0
    t1 = 50
    t_span = (t0, t1)
    t_eval = np.linspace(t0, t1, 100)

    # Solve the system
    def event(t, state):
        return state[1]
    event.terminal = True  # Stop the integration when the event is triggered
    event.direction = -1   # 0 means any crossing direction (both positive and negative)

    sol = solve_ivp(system, t_span, state0, t_eval=t_eval, events=event)

    # Plot the results
    plt.plot(sol.t, sol.y[0], label="x(t)")  # x(t) solution
    plt.plot(sol.t, sol.y[1], label="y(t)")  # y(t) solution
    plt.xlabel('Time t')
    plt.ylabel('Solution')
    plt.legend()
    plt.title('Solution of the System of ODEs')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()
