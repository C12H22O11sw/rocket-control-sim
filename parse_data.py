from scipy.interpolate import CloughTocher2DInterpolator
import numpy as np
from scipy.interpolate import interp1d
import pandas as pd

def parse_prop_data(filename):
    with open(filename, 'r') as file:
        data = file.read()
        pages = data.split('PROP RPM =')
        my_dict = {}
        for page in pages[1:]:
            lines = page.split('\n')
            rpm = int(lines[0])
            my_dict[rpm] = {}
            headers = lines[2].split()
            for line in lines[4:]:
                if line.isspace() or line == "":
                    continue
                values = line.split()
                if len(values) != len(headers):
                    print(f'Warning: Could not parse "{line}"\nskipping to next line')
                    continue
                vel = float(values[0])
                my_dict[rpm][vel] = {}
                for value, header in zip(values, headers):
                    if header == values[0]:
                        continue
                    my_dict[rpm][vel][header] = value
    return my_dict

def generate_bla(filename, attribute):
    data = parse_prop_data(filename)
    points = []
    values = []
    for rpm, page in data.items():
        for vel, line in page.items():
            points.append([rpm, vel])
            values.append(line[attribute])
    points = np.asarray(points)
    values = np.asarray(values)
    interp = CloughTocher2DInterpolator(points, values, fill_value=0)
    return interp

def func_from_csv(filename: str, x_label: str, y_label:str):
    print(f'Parsing data from {filename}')
    df = pd.read_csv(filename)
    x = df[x_label]
    y = df[y_label]

    interp = interp1d(x, y, kind='cubic')
    min_x = min(x)
    max_x = max(x)

    return (min_x, max_x), interp

def main():
    func = generate_bla('data/PER3_6x6E.dat', 'Thrust')
    print(func([10000, 20]))

if __name__ == '__main__':
    main()