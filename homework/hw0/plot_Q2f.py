import matplotlib.pyplot as plt
import numpy as np

# Load data
data_f_i = np.loadtxt("q2-f-i.txt", skiprows=1)
data_f_ii = np.loadtxt("q2-f-ii.txt", skiprows=1)

# Part i: θ3 sweep
theta3 = data_f_i[:, 0]
G1_i = data_f_i[:, 1]
G2_i = data_f_i[:, 2]
G3_i = data_f_i[:, 3]

plt.figure()
plt.plot(np.degrees(theta3), G1_i, label='G1')
plt.plot(np.degrees(theta3), G2_i, label='G2')
plt.plot(np.degrees(theta3), G3_i, label='G3')
plt.xlabel('θ3 (degrees)')
plt.ylabel('Gravity Vector Components')
plt.title('Gravity Vector vs θ3 (θ1 = 0°, d2 = 0.5 m)')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Part ii: d2 sweep
d2 = data_f_ii[:, 0]
G1_ii = data_f_ii[:, 1]
G2_ii = data_f_ii[:, 2]
G3_ii = data_f_ii[:, 3]

plt.figure()
plt.plot(d2, G1_ii, label='G1')
plt.plot(d2, G2_ii, label='G2')
plt.plot(d2, G3_ii, label='G3')
plt.xlabel('d2 (m)')
plt.ylabel('Gravity Vector Components')
plt.title('Gravity Vector vs d2 (θ1 = 0°, θ3 = 0°)')
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.show()
