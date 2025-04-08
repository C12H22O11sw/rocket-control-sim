from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt
from parse_data import func_from_csv
from rocket_dynamics import generate_dynamics, generate_control_plant_prop, generate_proportional_control_plant, appogee, generate_cd_func


# Main function to set up and solve the system
def main():

    """Main function to set up and solve the rocket dynamics system."""

    # Load drag coefficient and air density functions from CSV files
    cd_func = generate_cd_func('data/cd_function.csv', 'Airspeed', 'CD')
    _, density_func = func_from_csv('data/international_standard_atmosphere.csv', 'Altitude', 'Density')
    
    # Generate a proportional controller for the propulsion system
    control_func = generate_proportional_control_plant(30e3, 30e3, 0)

    # Generate the control plant for the propulsion system
    control_plant = generate_control_plant_prop('data/PER3_6x6E.dat', 4)

    # Define rocket parameters
    area = np.pi / 4 * (5.63e-2)**2  # Cross-sectional area (m^2)
    mass = 268 * 1e-3  # Mass (kg)


    """Configure and run simulation."""

    # Generate the system dynamics
    system = generate_dynamics(area, mass, cd_func, density_func, control_func, control_plant)

    # Initial conditions
    state0 = [0, 90]  # Initial position (m) and velocity (m/s)

    # Time span for simulation
    t0, t1 = 0, 50  # Start and end times (s)
    t_span = (t0, t1)
    t_eval = np.linspace(t0, t1, 100)  # Time points for evaluation

    # Solve the system of ODEs
    sol = solve_ivp(system, t_span, state0, t_eval=t_eval, events=appogee)


    """Plot the results."""

    plt.plot(sol.t, sol.y[0], label="Position (x(t))")  # Position vs. time
    plt.plot(sol.t, sol.y[1], label="Velocity (v(t))")  # Velocity vs. time
    plt.xlabel('Time (s)')
    plt.ylabel('State')
    plt.legend()
    plt.title('Rocket Dynamics Simulation')
    plt.grid(True)
    #plt.show()

# Entry point of the script
if __name__ == '__main__':
    main()
