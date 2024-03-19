import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import csv

# Read data from CSV file
data_points = []
with open('data_points.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)  # Skip header
    for row in csvreader:
        data_points.append([float(row[0]), float(row[1])])

data_points = np.array(data_points)
x_data = data_points[:, 0]
y_data = data_points[:, 1]

# Define the function for the polynomial fit
def polynomial_func(x, *coefficients):
    y = 0
    for i, coef in enumerate(coefficients):
        y += coef * x**i
    return y

# Initial guess for polynomial coefficients
initial_guess = np.ones(8)  # For 5th order polynomial, there are 6 coefficients (including constant)

# Perform the polynomial fitting
coefficients, covariance = curve_fit(polynomial_func, x_data, y_data, p0=initial_guess)

# Print the coefficients of the fitted polynomial
print("Coefficients of the fitted polynomial:", coefficients)

# Generate points for the fitted curve
x_fit = np.linspace(min(x_data), max(x_data), 100)
y_fit = polynomial_func(x_fit, *coefficients)

print(covariance)

# Plot the original data and the fitted curve
plt.plot(x_data, y_data, 'bo', label='Original Data')
plt.plot(x_fit, y_fit, 'r-', label='Fitted Curve (7th Order)')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Polynomial Fitting')
plt.legend()
plt.grid(True)
plt.show()
