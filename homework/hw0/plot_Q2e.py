import matplotlib.pyplot as plt
import numpy as np

# Load data
data_i = np.loadtxt("q2-e-i.txt", skiprows=1)
data_ii = np.loadtxt("q2-e-ii.txt", skiprows=1)

# Part i: θ3 sweep
theta3 = data_i[:, 0]
m11_i = data_i[:, 1]
m22_i = data_i[:, 2]
m33_i = data_i[:, 3]

plt.figure()
plt.plot(np.degrees(theta3), m11_i, label='m11')
plt.plot(np.degrees(theta3), m22_i, label='m22')
plt.plot(np.degrees(theta3), m33_i, label='m33')
plt.xlabel('θ3 (degrees)')
plt.ylabel('Mass Matrix Diagonal Elements')
plt.title('Mass Matrix vs θ3 (θ1 = 0°, d2 = 0.5 m)')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Part ii: d2 sweep
d2 = data_ii[:, 0]
m11_ii = data_ii[:, 1]
m22_ii = data_ii[:, 2]
m33_ii = data_ii[:, 3]

plt.figure()
plt.plot(d2, m11_ii, label='m11')
plt.plot(d2, m22_ii, label='m22')
plt.plot(d2, m33_ii, label='m33')
plt.xlabel('d2 (m)')
plt.ylabel('Mass Matrix Diagonal Elements')
plt.title('Mass Matrix vs d2 (θ1 = 0°, θ3 = 0°)')
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.show()
