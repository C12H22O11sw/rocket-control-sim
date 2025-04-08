from parse_data import generate_prop_function, func_from_csv
from scipy.constants import mile, hour, g, foot, mach
from scipy.integrate import solve_ivp
import numpy as np

def generate_cd_func(filename, vel_label, cd_label, unit='m/s'):
    if unit == 'm/s': conversion_factor = 1
    elif unit == 'ft/s': conversion_factor = foot
    elif unit == 'mach': conversion_factor = mach
    elif unit == 'km/h': conversion_factor = 1000 / hour
    elif unit == 'mi/h': conversion_factor = mile / hour
    else: raise ValueError(f"Unknown unit: {unit}")

    (min_cd, max_cd), base_func = func_from_csv(filename, vel_label, cd_label)
    def cd_func(vel):
        """
        Computes the drag coefficient based on velocity.
        """
        # Convert velocity to the appropriate unit
        vel = vel * conversion_factor

        # Calculate the drag coefficient using the base function
        cd = base_func(vel)

        # Ensure the drag coefficient is within the specified range
        cd = max(min_cd, min(cd, max_cd))
        return cd

    # Return the drag coefficient function
    return cd_func
        

# Function to generate the system dynamics
def simulate_rocket_prop(initial_state, rocket_config, environment_config, control_config=None, t_max=10, t_eval=None):
    """
    Generates the system of ODEs representing the rocket's dynamics.
    """

    area = rocket_config['area']  # Cross-sectional area (m^2)
    mass = rocket_config['mass']  # Mass (kg)
    cd_func = rocket_config['cd_func']  # Drag coefficient function

    density_func = environment_config['density_func']  # Air density function

    control_func = control_config['control_func']  # Control function
    control_plant = control_config['control_plant']  # Control plant function

    if t_eval is None:
        t_eval = np.linspace(0, t_max, 1000)
    t_span = (0, t_max)  # Time span for simulation

    accel = 0  # Initialize acceleration
    observations = {'Time': [],
                    'Drag': [],
                    'Thrust': [],
                    'Density': [],
                    'Control': [],
                    'Position': [],
                    'Velocity': [],
                    'Acceleration': []}  # Initialize observations

    def system(t, state, observe=False):
        """
        Computes the derivatives of position and velocity.
        """
        pos, vel = state
        nonlocal accel
        nonlocal observations

        # Calculate drag force
        cd = cd_func(vel)
        density = density_func(pos)
        drag = 0.5 * density * cd * area * vel**2

        # Calculate net force
        control_input = 0
        thrust = 0
        if control_func is not None and control_plant is not None:
            signal = pos, vel, accel
            control_input = control_func(t, signal)
            thrust = control_plant(control_input, vel)
        net_force = thrust - drag

        # Update acceleration
        accel = net_force / mass - g  # Subtract gravitational acceleration

        if observe:
            """Record observations."""
            observations['Time'].append(t)
            observations['Drag'].append(drag)
            observations['Thrust'].append(thrust)
            observations['Density'].append(density)
            observations['Control'].append(control_input)
            observations['Position'].append(pos)
            observations['Velocity'].append(vel)
            observations['Acceleration'].append(accel)

        return vel, accel
    
    sol = solve_ivp(system, t_span, initial_state, t_eval=t_eval, events=appogee)

    # Record observations at each time step
    for t, state in zip(sol.t, sol.y.T):
        system(t, state, observe=True)

    # Convert observations to numpy arrays
    for key in observations:
        observations[key] = np.asarray(observations[key])

    return observations

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
        force = num_props * thrust_function([rpm, vel])[0]
        return force

    return plant

def generate_control_plant_drag_flaps(filename, num_flaps=1):
    """
    Generates a control plant function for the drag flaps based on the given data file.
    """
    drag_function = fun(filename, 'Drag')

    def plant(rpm, vel):
        """
        Computes the drag force based on RPM and velocity.
        """
        force = num_flaps * drag_function([rpm, vel])[0]
        return force

    return plant

def generate_proportional_control_plant(gain, max_val, min_val=0, target=0):
    """
    Generates a control plant function for the propulsion system.
    """
    # Define the control function
    def control_func(t, signal):
        """
        Computes the control input based on the current state.
        """
        _, _, accel = signal
        control_output = gain * (target - accel)
        control_output = max(control_output, min_val)
        control_output = min(control_output, max_val)
        return control_output
    return control_func

def generate_pid_control_plant(gain, max_val, min_val=0, target=0):
    """
    Generates a control plant function for the propulsion system.
    """
    kp, ki, kd = gain
    t_old = 0
    integral = 0
    derivative = 0
    # Define the control function
    def control_func(t, signal):
        """
        Computes the control input based on the current state.
        """
        _, _, accel = signal
        nonlocal t_old, integral, derivative
        dt = t - t_old
        t_old = t
        error = target - accel
        if dt > 0:      
            integral += error * dt
            derivative = (error - derivative) / dt
        control_output = kp * error + ki * integral + kd * derivative
        control_output = max(control_output, min_val)
        control_output = min(control_output, max_val)
        return control_output
    return control_func

def generate_trivial_controller(value):
    """
    Generates a trivial control plant function that returns a constant value.
    """
    def control_func(t, signal):
        """
        Returns a constant control output.
        """
        return value
    return control_func

# Define an event to stop the simulation when velocity becomes zero
def appogee(t, state):
    _, vel = state
    return vel  # Trigger when velocity is zero
appogee.terminal = True  # Stop the integration when the event is triggered
appogee.direction = -1   # Only trigger when velocity decreases to zero