import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file into a pandas DataFrame
df = pd.read_csv("csvs/esti.csv")
# print(df)
df = df.iloc[10:800]

# Create subplots
fig, axs = plt.subplots(6, 1, figsize=(20,11))

# Plot x coordinates
axs[0].plot(df['t'], df['px_est'], label='x_estimation', color='blue', linewidth=2.0)
axs[0].plot(df['t'], df['px_gt'], label='x_ground_truth', color='red',linestyle='--', linewidth=2.0)
axs[0].set_ylabel('x (m)')
axs[0].set_ylim([-2.5, 2.5])
axs[0].legend(loc='upper right')

# Plot y coordinates
axs[1].plot(df['t'], df['py_est'], label='y_estimation', color='blue', linewidth=2.0)
axs[1].plot(df['t'], df['py_gt'], label='y_ground_truth', color='red',linestyle='--', linewidth=2.0)
axs[1].set_ylabel('y (m)')
axs[1].set_ylim([-2.5, 2.5])
axs[1].legend(loc='upper right')

# Plot z coordinates
axs[2].plot(df['t'], -df['pz_est'], label='z_estimation', color='blue', linewidth=2.0)
axs[2].plot(df['t'], -df['pz_gt'], label='z_ground_truth', color='red',linestyle='--', linewidth=2.0)
axs[2].set_ylabel('z (m)')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylim([19.5, 20.5])
axs[2].legend(loc='upper right')

# Plot rr
axs[3].plot(df['t'], df['rr_est'], label='rr_estimation', color='blue', linewidth=2.0)
axs[3].plot(df['t'], df['rr_gt'], label='rr_ground_truth', color='red',linestyle='--', linewidth=2.0)
axs[3].set_ylabel('Roll (rad)')
axs[3].set_ylim([-0.2, 0.2])
axs[3].legend(loc='upper right')

# Plot rp
axs[4].plot(df['t'], df['rp_est'], label='rp_estimation', color='blue', linewidth=2.0)
axs[4].plot(df['t'], df['rp_gt'], label='rp_ground_truth', color='red',linestyle='--', linewidth=2.0)
axs[4].set_ylabel('Pitch (rad)')
axs[4].set_ylim([-0.2, 0.2])
axs[4].legend(loc='upper right')

# Plot ry
axs[5].plot(df['t'], df['ry_est'], label='ry_estimation', color='blue', linewidth=2.0)
axs[5].plot(df['t'], df['ry_gt'], label='ry_ground_truth', color='red',linestyle='--', linewidth=2.0)
axs[5].set_ylabel('Yaw (rad)')
axs[5].set_ylim([-3.5, 3.5])
axs[5].legend(loc='upper right')

# Add title to the entire figure
# fig.suptitle('Estimation vs Ground Truth')

plt.savefig("figs/pos.pdf", bbox_inches='tight')


# Show plot
plt.show()
