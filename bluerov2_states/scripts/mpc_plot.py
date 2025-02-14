import pandas as pd
import matplotlib.pyplot as plt

# Load the first CSV data
df_ampc = pd.read_csv("csvs/ampc1700.csv")
df_mpc = pd.read_csv("csvs/mpc1700.csv")
df_pid = pd.read_csv("csvs/pid.csv")
# df_pid = pd.read_csv("csvs/pid.csv")

# Remove leading spaces from column names
df_ampc.columns = df_ampc.columns.str.strip()

# Extract relevant columns from the first CSV
time1 = df_ampc['t']
ampc_gt = df_ampc['pz_gt']
pz_ref = df_mpc['pz_ref']
mpc_gt = df_mpc['pz_gt']
pid_gt = df_pid['pz_gt']

# Extract relevant columns from the second CSV
time2 = df_mpc['t']

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(time1, (ampc_gt), label='ampc', linestyle='-', linewidth=6.0)
plt.plot(time1, (mpc_gt), label='mpc', linestyle='-', linewidth=6.0)
plt.plot(time1, (pid_gt[:600]), label='pid', linestyle='-', linewidth=6.0)
plt.plot(time1, pz_ref, color = 'black', label='ref', linestyle='--', linewidth=4.0)
# [:600]
# Add labels and title
plt.xlabel('Time')
plt.ylabel('x (m)')
# plt.ylim(-2, 6)
# plt.title('Comparison of Estimated Velocities and Ground Truth over Time')
plt.legend()

# Show plot
plt.grid(True)
plt.show()
