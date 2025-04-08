from parse_data import generate_prop_function, func_from_csv
from scipy.constants import mile, hour, g, foot, mach

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
        force = num_props * thrust_function([rpm, vel * (hour / mile)])[0]
        return force

    return plant

def generate_proportional_control_plant(gain, max_val, min_val=0):
    """
    Generates a control plant function for the propulsion system.
    """
    # Define the control function
    def control_func(t, signal):
        """
        Computes the control input based on the current state.
        """
        _, _, accel = signal
        control_output = gain * accel
        control_output = max(control_output, min_val)
        control_output = min(control_output, max_val)
        return control_output
    return control_func

# Define an event to stop the simulation when velocity becomes zero
def appogee(t, state):
    _, vel = state
    return vel  # Trigger when velocity is zero
appogee.terminal = True  # Stop the integration when the event is triggered
appogee.direction = -1   # Only trigger when velocity decreases to zero