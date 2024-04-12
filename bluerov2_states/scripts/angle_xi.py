import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file into a pandas DataFrame
df = pd.read_csv("csvs/esti.csv")

df = df.iloc[10:800]

# Create subplots
fig, axs = plt.subplots(6, 1, figsize=(20, 11))

# Plot x coordinates
axs[0].plot(df['t'], df['vx_est'], label='vx_est', color='blue', linewidth=2.0)
axs[0].plot(df['t'], df['vx_gt'], label='vx_gt', color='red', linewidth=2.0)
axs[0].set_ylabel('vx (m/s)')
# axs[0].set_ylim([-2.5, 2.5])
axs[0].legend(loc='upper right')

# Plot y coordinates
axs[1].plot(df['t'], df['vy_est'], label='vy_est', color='blue', linewidth=2.0)
axs[1].plot(df['t'], df['vy_gt'], label='vy_gt', color='red', linewidth=2.0)
axs[1].set_ylabel('vy (m/s)')
# axs[1].set_ylim([-2.5, 2.5])
axs[1].legend(loc='upper right')

# Plot z coordinates
axs[2].plot(df['t'], -df['vz_est'], label='vz_est', color='blue', linewidth=2.0)
axs[2].plot(df['t'], -df['vz_gt'], label='vz_gt', color='red', linewidth=2.0)
axs[2].set_ylabel('vz (m/s)')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylim([-2.0, 2.0])
axs[2].legend(loc='upper right')



# Smooth out est_x using a moving average
window_size = 5  # Adjust window size as needed
df['smoothed_gt_x'] = df['x_gt'].rolling(window=window_size, min_periods=1).mean()
df['smoothed_est_y'] = df['y_est'].rolling(window=window_size, min_periods=1).mean()
df['smoothed_gt_y'] = df['y_gt'].rolling(window=window_size, min_periods=1).mean()
df['smoothed_gt_z'] = df['z_gt'].rolling(window=window_size, min_periods=1).mean()

# Plot rr
axs[3].plot(df['t'], df['x_est']*10, label='dist_x_est', color='blue', linewidth=2.0)
axs[3].plot(df['t'], df['smoothed_gt_x']*10, label='dist_x_gt', color='red', linewidth=2.0)
axs[3].set_ylabel('dist_x (N)')
axs[3].set_ylim([-10, 20])
axs[3].legend(loc='upper right')

# Plot rp
axs[4].plot(df['t'], df['smoothed_est_y']*10, label='dist_y_est', color='blue', linewidth=2.0)
axs[4].plot(df['t'], df['smoothed_gt_y']*10, label='dist_y_gt', color='red', linewidth=2.0)
axs[4].set_ylabel('dist_y (N)')
axs[4].set_ylim([-10, 20])
axs[4].legend(loc='upper right')

# Plot ry
axs[5].plot(df['t'], df['z_est']*10, label='dist_z_est', color='blue', linewidth=2.0)
axs[5].plot(df['t'], df['smoothed_gt_z']*10, label='dist_z_gt', color='red', linewidth=2.0)
axs[5].set_ylabel('dist_z (N)')
axs[5].set_ylim([-10, 20])
axs[5].legend(loc='upper right')

# Add title to the entire figure
# fig.suptitle('est vs Ground Truth')

plt.savefig("figs/angle_xi.pdf", bbox_inches='tight')


# Show plot
plt.show()
