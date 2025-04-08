from scipy.interpolate import CloughTocher2DInterpolator
import numpy as np
from scipy.interpolate import interp1d
import pandas as pd

def parse_prop_data(filename):
    """
    Parses propulsion data from a file and organizes it into a nested dictionary.
    """
    with open(filename, 'r') as file:
        data = file.read()
        pages = data.split('PROP RPM =')
        prop_data = {}

        for page in pages[1:]:
            lines = page.split('\n')
            rpm = int(lines[0])  # Extract RPM value
            prop_data[rpm] = {}

            headers = lines[2].split()  # Extract column headers
            for line in lines[4:]:
                if line.isspace() or line == "":
                    continue  # Skip empty lines

                values = line.split()
                if len(values) != len(headers):
                    print(f'Warning: Could not parse "{line}"\nskipping to next line')
                    continue

                velocity = float(values[0])  # First value is velocity
                prop_data[rpm][velocity] = {}

                for value, header in zip(values, headers):
                    if header == values[0]:  # Skip the velocity column
                        continue
                    prop_data[rpm][velocity][header] = value

    return prop_data

def generate_prop_function(filename, attribute):
    """
    Generates a 2D interpolator for a specific attribute from propulsion data.
    """
    data = parse_prop_data(filename)
    points = [[0, 0]]  # Initialize with a point for zero velocity
    values = [0]  # Initialize with a value for zero velocity

    for rpm, rpm_data in data.items():
        for velocity, attributes in rpm_data.items():
            points.append([rpm, velocity])
            values.append(attributes[attribute])

    points = np.asarray(points)
    values = np.asarray(values)

    # Create a 2D interpolator
    interpolator = CloughTocher2DInterpolator(points, values, fill_value=0)

    return interpolator

def func_from_csv(filename: str, x_label: str, y_label: str):
    """
    Reads data from a CSV file and creates a 1D cubic interpolator.
    """
    print(f'Parsing data from {filename}')
    df = pd.read_csv(filename)

    x = df[x_label]
    y = df[y_label]

    # Create a 1D cubic interpolator
    interpolator = interp1d(x, y, kind='cubic')
    min_x = min(x)
    max_x = max(x)

    return (min_x, max_x), interpolator

def main():
    """
    Main function to demonstrate the usage of generate_bla.
    """
    interpolator = generate_prop_function('data/PER3_6x6E.dat', 'Thrust')
    print(interpolator([10000, 20]))  # Example usage

if __name__ == '__main__':
    main()