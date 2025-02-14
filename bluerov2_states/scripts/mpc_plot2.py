import pandas as pd
import matplotlib.pyplot as plt

# Load the first CSV data
df_ampc = pd.read_csv("csvs/ampc_w_real_40.csv")
df_mpc = pd.read_csv("csvs/mpc_base_40.csv")

# Remove leading spaces from column names
df_ampc.columns = df_ampc.columns.str.strip()

# Extract relevant columns from the first CSV
time1 = df_ampc['t']
ampc_gt = df_ampc['py_gt']
px_ref = df_ampc['py_ref']
mpc_gt = df_mpc['py_gt']

# Extract relevant columns from the second CSV
time2 = df_mpc['t']


# Plotting
plt.figure(figsize=(10, 6))
plt.plot(time1, (ampc_gt), label='ampc', linestyle='-')
plt.plot(time1, (mpc_gt[:600]), label='mpc', linestyle='-')
plt.plot(time1, px_ref, label='ref', linestyle='-')

# Add labels and title
plt.xlabel('Time')
plt.ylabel('x (m)')
# plt.ylim(-2, 6)
# plt.title('Comparison of Estimated Velocities and Ground Truth over Time')
plt.legend()

# Show plot
plt.grid(True)
plt.show()
