import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d, CubicSpline

# distance from speaker relative to camera in meters
x = np.array([1, 6, 7, 9, 12, 20])

# shooter velocity in native units (RPMs)
y = np.array([2, 8, 6, 10, 14, 41])

# Original data points
plt.plot(x, y, "o", label="Data Points")

# Generating a finer x-axis for plotting interpolations
x_fine = np.linspace(x.min(), x.max(), 300)

# Linear spline interpolation
# linear_interp = interp1d(x, y, kind="linear")
# plt.plot(x_fine, linear_interp(x_fine), "-", label="Linear Interpolation")

# Quadratic spline interpolation
# quadratic_interp = interp1d(x, y, kind="quadratic")
# plt.plot(x_fine, quadratic_interp(x_fine), "--", label="Quadratic Interpolation")

# Cubic spline interpolation
# cubic_interp = interp1d(x, y, kind="cubic")
# plt.plot(x_fine, cubic_interp(x_fine), ":", label="Cubic Interpolation")

# Cubic spline interpolation with boundary coundition
cubic_interp_bc = CubicSpline(x, y, bc_type="natural")
plt.plot(
    x_fine,
    cubic_interp_bc(x_fine),
    ":",
    label="Cubic Interpolation with natural boundary condition",
)

# Example x value
x_value = 3.1415

# Predicting the y value using cubic spline interpolation
y_predicted = cubic_interp_bc(x_value)

print(f"Predicted shooter velocity at {x_value} meters: {y_predicted} units")


# Labeling the x-axis and y-axis
plt.xlabel("Distance from Speaker (meters)")
plt.ylabel("Shooter Velocity (RPMs)")

plt.legend()
plt.show()
