from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt
import yaml
import argparse
from parse_data import func_from_csv
from rocket_dynamics import *


# Main function to set up and solve the system
def main():
    
    """
    parser = argparse.ArgumentParser(description="Rocket Dynamics Simulation")
    parser.add_argument('--config', type=str, default='config.yaml', help='Path to the configuration file')
    args = parser.parse_args()
    config_file = args.config
    with open(config_file, 'r') as file:
        defalt_config = yaml.safe_load(file)
    
    parser.set_defaults(**defalt_config)
    parser.add_argument('--diameter')
    parser.add_argument('--mass')
    parser.add_argument('--control method')   """  

    """Main function to set up and solve the rocket dynamics system."""

    # Load drag coefficient and air density functions from CSV files
    cd_func = generate_cd_func('data/cd_function.csv', 'Airspeed', 'CD')
    _, density_func = func_from_csv('data/international_standard_atmosphere.csv', 'Altitude', 'Density')
    
    # Generate a proportional controller for the propulsion system
    gain = (30e3, 3e3, -0e0)  # Proportional gain
    min_val = 0  # Minimum control output
    max_val = 30e3  # Maximum control output
    #control_func = generate_pid_control_plant(gain, max_val, min_val, -g)
    control_func = generate_proportional_control_plant(10e3, max_val, min_val, -g)

    # Generate the control plant for the propulsion system
    control_plant = generate_control_plant_prop('data/PER3_6x6E.dat', 4)

    # Define rocket parameters
    area = np.pi / 4 * (5.63e-2)**2  # Cross-sectional area (m^2)
    mass = 2680 * 1e-3  # Mass (kg)


    """Configure and run simulation."""

    rocket_config = {
        'area': area,
        'mass': mass,
        'cd_func': cd_func
    }
    environment_config = {
        'density_func': density_func
    }
    control_config = {
        'control_func': control_func,
        'control_plant': control_plant
    }

    observations = simulate_rocket_prop([0, 90], rocket_config, environment_config, control_config)
    time = observations['Time']
    position = observations['Position']
    velocity = observations['Velocity']
    acceleration = observations['Acceleration']
    thrust = observations['Thrust']
    drag = observations['Drag']

    """Plot the results."""

    plt.plot(time, acceleration, label="Acceration (x(t))")  # Position vs. time
    plt.plot(time, thrust, label="Thrust (v(t))")  # Velocity vs. time
    plt.plot(time, drag, label="Drag (v(t))")  # Velocity vs. time
    #plt.plot(time, velocity, label="Velocity (v(t))")  # Velocity vs. time
    #plt.plot(time, position, label="Position (v(t))")  # Velocity vs. time
    plt.xlabel('Time (s)')
    plt.ylabel('State')
    plt.legend()
    plt.title('Rocket Dynamics Simulation')
    plt.grid(True)
    plt.show()

# Entry point of the script
if __name__ == '__main__':
    main()
