#!/usr/bin/env python3

import sys
import numpy as np
import matplotlib.pyplot as plt

def read_text_file(filename):
    data = []
    with open(filename, 'r') as file:
        for line in file:
            try:
                line_data = [float(value) for value in line.strip().split(',')]
            except ValueError:
                pass
            else:
                data.append(line_data)
    return np.array(data)

def main(args):
    filename = args[1]
    array_data = read_text_file(filename)
    # t = np.linspace(0, 150, len(array_data[:,]))
    # array_data = np.c_[t, array_data]
    plt.plot(array_data)
    plt.show()

def save_to_csv(data):
    np.savetxt("data.csv", data, delimiter=",")
    print("data saved to csv!")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python script.py <filename>")
        sys.exit(1)

    main(sys.argv)
