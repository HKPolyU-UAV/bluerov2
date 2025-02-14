import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file into a pandas DataFrame
df_ampc = pd.read_csv("csvs/ampc1700.csv")
df_mpc = pd.read_csv("csvs/mpc1700.csv")
df_pid = pd.read_csv("csvs/pid.csv")
df_pid = df_pid.iloc[0:600]


# Create subplots
fig, axs = plt.subplots(3, 1, figsize=(20, 11))

# ampc_error_x = df_ampc['px_gt']-df_mpc['px_ref']
# mpc_error_x = df_mpc['px_gt']-df_mpc['px_ref']
# pid_error_x = df_pid['px_gt']-df_mpc['px_ref']

# Plot x coordinates
axs[0].plot(df_mpc['t'], df_ampc['px_gt']-df_mpc['px_ref'], label='ampc', color='blue', linewidth=2.0)
axs[0].plot(df_mpc['t'], df_mpc['px_gt']-df_mpc['px_ref'], label='mpc', color='orange', linewidth=2.0)
axs[0].plot(df_mpc['t'], df_pid['px_gt']-df_mpc['px_ref'], label='pid', color='red', linewidth=2.0)
axs[0].plot(df_mpc['t'], df_mpc['px_ref']-df_mpc['px_ref'], label='ref', color='black',linestyle='--', linewidth=2.0)
axs[0].set_ylabel('x (m)')
# axs[0].set_ylim([-2.8, 3.2])
axs[0].legend(loc='upper right')

# Plot y coordinates
axs[1].plot(df_mpc['t'], df_ampc['py_gt']-df_mpc['py_ref'], label='ampc', color='blue', linewidth=2.0)
axs[1].plot(df_mpc['t'], df_mpc['py_gt']-df_mpc['py_ref'], label='mpc', color='orange', linewidth=2.0)
axs[1].plot(df_mpc['t'], df_pid['py_gt']-df_mpc['py_ref'], label='pid', color='red', linewidth=2.0)
axs[1].plot(df_mpc['t'], df_mpc['py_ref']-df_mpc['py_ref'], label='ref', color='black',linestyle='--', linewidth=2.0)
axs[1].set_ylabel('y (m)')
# axs[1].set_ylim([-2.8, 3.2])
axs[1].legend(loc='upper right')

# Plot z coordinates
# Plot y coordinates
axs[2].plot(df_mpc['t'], -df_ampc['pz_gt']--df_mpc['pz_ref'], label='ampc', color='blue', linewidth=2.0)
axs[2].plot(df_mpc['t'], -df_mpc['pz_gt']--df_mpc['pz_ref'], label='mpc', color='orange', linewidth=2.0)
axs[2].plot(df_mpc['t'], -df_pid['pz_gt']--df_mpc['pz_ref'], label='pid', color='red', linewidth=2.0)
axs[2].plot(df_mpc['t'], -df_mpc['pz_ref']--df_mpc['pz_ref'], label='ref', color='black',linestyle='--', linewidth=2.0)
axs[2].legend(loc='upper right')
axs[2].set_ylabel('z (m)')
axs[2].set_xlabel('Time (s)')
# axs[2].set_ylim([18.5, 20.5])
axs[2].legend(loc='upper right')

# Add title to the entire figure
# fig.suptitle('Estimation vs Ground Truth')

plt.savefig("figs/error.pdf", bbox_inches='tight')

# Show plot
plt.show()
