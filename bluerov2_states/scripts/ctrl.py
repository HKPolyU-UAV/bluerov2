import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file into a pandas DataFrame
df_ampc = pd.read_csv("csvs/ampc1700.csv")
df_mpc = pd.read_csv("csvs/mpc1700.csv")
df_pid = pd.read_csv("csvs/pid.csv")
df_pid = df_pid.iloc[0:600]


# Create subplots
fig, axs = plt.subplots(3, 1, figsize=(20,11))

# Plot x coordinates
axs[0].plot(df_mpc['t'], df_ampc['px_gt'], label='ampc', color='blue', linewidth=2.0)
axs[0].plot(df_mpc['t'], df_mpc['px_gt'], label='mpc', color='orange', linewidth=2.0)
axs[0].plot(df_mpc['t'], df_pid['px_gt'], label='pid', color='red', linewidth=2.0)
axs[0].plot(df_mpc['t'], df_mpc['px_ref'], label='ref', color='black',linestyle='--', linewidth=2.0)
axs[0].set_ylabel('x (m)')
axs[0].set_ylim([-2.8, 3.2])
axs[0].legend(loc='upper right')

# Plot y coordinates
axs[1].plot(df_mpc['t'], df_ampc['py_gt'], label='ampc', color='blue', linewidth=2.0)
axs[1].plot(df_mpc['t'], df_mpc['py_gt'], label='mpc', color='orange', linewidth=2.0)
axs[1].plot(df_mpc['t'], df_pid['py_gt'], label='pid', color='red', linewidth=2.0)
axs[1].plot(df_mpc['t'], df_mpc['py_ref'], label='ref', color='black',linestyle='--', linewidth=2.0)
axs[1].set_ylabel('y (m)')
axs[1].set_ylim([-2.8, 3.2])
axs[1].legend(loc='upper right')

# Plot z coordinates
# Plot y coordinates
axs[2].plot(df_mpc['t'], -df_ampc['pz_gt'], label='ampc', color='blue', linewidth=2.0)
axs[2].plot(df_mpc['t'], -df_mpc['pz_gt'], label='mpc', color='orange', linewidth=2.0)
axs[2].plot(df_mpc['t'], -df_pid['pz_gt'], label='pid', color='red', linewidth=2.0)
axs[2].plot(df_mpc['t'], -df_mpc['pz_ref'], label='ref', color='black',linestyle='--', linewidth=2.0)
axs[2].legend(loc='upper right')
axs[2].set_ylabel('z (m)')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylim([18.5, 20.5])
axs[2].legend(loc='upper right')

# Add title to the entire figure
# fig.suptitle('Estimation vs Ground Truth')

plt.savefig("figs/ctrl.pdf", bbox_inches='tight')

# Show plot
plt.show()
