import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file into a pandas DataFrame
df = pd.read_csv("csvs/esti.csv")

df = df.iloc[10:800]

# Create subplots
fig, axs = plt.subplots(6, 1, figsize=(8, 10))

# Plot x coordinates
axs[0].plot(df['t'], df['vx_est'], label='v_estimation', color='blue', linewidth=1.0)
axs[0].plot(df['t'], df['vx_gt'], label='v_ground_truth', color='red', linewidth=1.0)
axs[0].set_ylabel('x (m/s)')
# axs[0].set_ylim([-2.5, 2.5])
axs[0].legend(loc='upper right')

# Plot y coordinates
axs[1].plot(df['t'], df['vy_est'], label='y_estimation', color='blue', linewidth=1.0)
axs[1].plot(df['t'], df['vy_gt'], label='y_ground_truth', color='red', linewidth=1.0)
axs[1].set_ylabel('y (m/s)')
# axs[1].set_ylim([-2.5, 2.5])
axs[1].legend(loc='upper right')

# Plot z coordinates
axs[2].plot(df['t'], -df['vz_est'], label='z_estimation', color='blue', linewidth=1.0)
axs[2].plot(df['t'], -df['vz_gt'], label='z_ground_truth', color='red', linewidth=1.0)
axs[2].set_ylabel('z (m/s)')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylim([-1.0, 1.0])
axs[2].legend(loc='upper right')



# Smooth out est_x using a moving average
window_size = 5  # Adjust window size as needed
df['smoothed_gt_x'] = df['x_gt'].rolling(window=window_size, min_periods=1).mean()
df['smoothed_est_y'] = df['y_est'].rolling(window=window_size, min_periods=1).mean()
df['smoothed_gt_y'] = df['y_gt'].rolling(window=window_size, min_periods=1).mean()
df['smoothed_gt_z'] = df['z_gt'].rolling(window=window_size, min_periods=1).mean()

# Plot rr
axs[3].plot(df['t'], df['x_est']*10, label='rr_estimation', color='blue', linewidth=1.0)
axs[3].plot(df['t'], df['smoothed_gt_x']*10, label='rr_ground_truth', color='red', linewidth=1.0)
axs[3].set_ylabel('x (N)')
axs[3].set_ylim([-10, 20])
axs[3].legend(loc='upper right')

# Plot rp
axs[4].plot(df['t'], df['smoothed_est_y']*10, label='rp_estimation', color='blue', linewidth=1.0)
axs[4].plot(df['t'], df['smoothed_gt_y']*10, label='rp_ground_truth', color='red', linewidth=1.0)
axs[4].set_ylabel('y (N)')
axs[4].set_ylim([-10, 20])
axs[4].legend(loc='upper right')

# Plot ry
axs[5].plot(df['t'], df['z_est']*10, label='ry_estimation', color='blue', linewidth=1.0)
axs[5].plot(df['t'], df['smoothed_gt_z']*10, label='ry_ground_truth', color='red', linewidth=1.0)
axs[5].set_ylabel('z (N)')
axs[5].set_ylim([-10, 20])
axs[5].legend(loc='upper right')

# Add title to the entire figure
# fig.suptitle('Estimation vs Ground Truth')

plt.savefig("figs/angle_xi.pdf", bbox_inches='tight')


# Show plot
plt.show()
