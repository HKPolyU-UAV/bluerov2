import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV data
df = pd.read_csv("csvs/esti.csv")

# Remove leading spaces from column names
df.columns = df.columns.str.strip()

# Extract relevant columns
time = df['t']
vx_est = df['x_est']
vx_gt = df['x_gt']

# print(vx_gt.size)

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(time, vx_est, label='vx_est', linestyle='-')
plt.plot(time, vx_gt, label='vx_gt', linestyle='-')

# print(vx_gt)

# Add labels and title
plt.xlabel('Time')
plt.ylabel('Velocity (vx)')
plt.ylim(-2,6)
plt.title('Estimated and Ground Truth Velocities over Time')
plt.legend()

# Show plot
plt.grid(True)
plt.show()
