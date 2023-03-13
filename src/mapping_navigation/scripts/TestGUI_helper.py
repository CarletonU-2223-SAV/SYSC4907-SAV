import math
import numpy as np
import matplotlib.pyplot as plt

# Define the radius
r = 33

# Define the number of points to generate
num_points = 5

# Generate the points on the curve
points = []
for i in range(num_points):
    theta = i * (-math.pi / 2 / num_points)  # Calculate the angle
    x = r * math.cos(theta)  # Calculate the x-coordinate
    y = r * math.sin(theta)  # Calculate the y-coordinate
    points.append([x, y])  # Add the point to the list

print(points)

data = np.array(points)
x, y = data.T

# plot our list in X,Y coordinates
plt.scatter(x, y)
plt.show()

