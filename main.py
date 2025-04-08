from scipy.integrate import solve_ivp
from scipy.constants import mile, hour, g
import numpy as np
import matplotlib.pyplot as plt
from parse_data import generate_prop_function, func_from_csv

# Function to generate a drag force function
def generate_drag_function(area: float, cd_func):
    """
    Generates a drag force function based on the given area and drag coefficient function.
    """
    drag = lambda state: 0.5 * cd_func(state) * area * state.density * state.vel**2
    return drag

# Function to generate the system dynamics
def generate_dynamics(area, mass, cd_func, density_func, control_func=None, control_plant=None):
    """
    Generates the system of ODEs representing the rocket's dynamics.
    """
    accel = 0  # Initialize acceleration

    def system(t, state):
        """
        Computes the derivatives of position and velocity.
        """
        pos, vel = state
        nonlocal accel

        # Calculate drag force
        cd = cd_func(vel)
        density = density_func(pos)
        drag = 0.5 * density * cd * area * vel**2

        # Calculate net force
        net_force = -drag
        if control_func is not None and control_plant is not None:
            signal = pos, vel, accel
            control_input = control_func(t, signal)
            net_force += control_plant(control_input, vel)

        # Update acceleration
        accel = net_force / mass - g  # Subtract gravitational acceleration
        return vel, accel

    return system

# Function to generate a control plant for the propulsion system
def generate_control_plant_prop(filename, num_props=1):
    """
    Generates a control plant function for the propulsion system based on the given data file.
    """
    thrust_function = generate_prop_function(filename, 'Thrust')

    def plant(rpm, vel):
        """
        Computes the thrust force based on RPM and velocity.
        """
        force = num_props * thrust_function([rpm, vel * (hour / mile)])
        return force[0]

    return plant

# Main function to set up and solve the system
def main():
    # Load drag coefficient and air density functions from CSV files
    (min_vel, max_vel), cd_func = func_from_csv('data/cd_function.csv', 'Airspeed', 'CD')
    (min_density, max_density), density_func = func_from_csv('data/international_standard_atmosphere.csv', 'Altitude', 'Density')

    # Define the control function
    def control_func(t, signal):
        """
        Computes the control input based on the current state.
        """
        pos, vel, accel = signal
        return min(30000 * accel, 30000)

    # Generate the control plant for the propulsion system
    control_plant = generate_control_plant_prop('data/PER3_6x6E.dat', 4)

    # Define rocket parameters
    area = np.pi / 4 * (5.63e-2)**2  # Cross-sectional area (m^2)
    mass = 268 * 1e-3  # Mass (kg)

    # Generate the system dynamics
    system = generate_dynamics(area, mass, cd_func, density_func, control_func, control_plant)

    # Initial conditions
    state0 = [0, 90]  # Initial position (m) and velocity (m/s)

    # Time span for simulation
    t0, t1 = 0, 50  # Start and end times (s)
    t_span = (t0, t1)
    t_eval = np.linspace(t0, t1, 100)  # Time points for evaluation

    # Define an event to stop the simulation when velocity becomes zero
    def event(t, state):
        return state[1]  # Trigger when velocity is zero
    event.terminal = True  # Stop the integration when the event is triggered
    event.direction = -1   # Only trigger when velocity decreases to zero

    # Solve the system of ODEs
    sol = solve_ivp(system, t_span, state0, t_eval=t_eval, events=event)

    # Plot the results
    plt.plot(sol.t, sol.y[0], label="Position (x(t))")  # Position vs. time
    plt.plot(sol.t, sol.y[1], label="Velocity (v(t))")  # Velocity vs. time
    plt.xlabel('Time (s)')
    plt.ylabel('State')
    plt.legend()
    plt.title('Rocket Dynamics Simulation')
    plt.grid(True)
    plt.show()

# Entry point of the script
if __name__ == '__main__':
    main()
