import numpy as np
import csv

# Define the function f(x)
def f(x):
    return np.abs(x) * x

# Generate data from -20 to 20
x_values = np.linspace(-20, 20, num=100000)
y_values = f(x_values)

# Create a list to store data points
data_points = []

# Iterate over each data point and store (x, y) pairs
for x, y in zip(x_values, y_values):
    data_points.append([x, y])

# Specify the filename
filename = "data_points.csv"

# Write the data to a CSV file
with open(filename, 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    # Write the header
    csvwriter.writerow(['x', 'y'])
    # Write the data points
    csvwriter.writerows(data_points)

print("Data saved to", filename)
