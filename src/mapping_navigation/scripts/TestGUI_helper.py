import math

# Define the radius
r = 33

# Define the number of points to generate
num_points = 20

# Generate the points on the curve
points = []
for i in range(num_points):
    theta = i * (math.pi / 2 / num_points)  # Calculate the angle
    x = r * math.cos(theta)  # Calculate the x-coordinate
    y = r * math.sin(theta)  # Calculate the y-coordinate
    points.append((x, y))  # Add the point to the list

# Print the points
print(points)

# importing Matplotlib and Numpy Packages
import numpy as np
import matplotlib.pyplot as plt

# The data are given as list of lists (2d list)
data = np.array([[33.0, 0.0], [32.89827201319322, 2.589150159018883], [32.59371523963954, 5.162337346327619], [32.088207373123325, 7.703697007244878], [31.384865037740067, 10.197560814373263], [30.48802457287246, 12.628553268047963], [29.40321529821614, 14.981686491405043], [28.137125423685045, 17.24245263562631], [26.697560814373265, 19.396913325651614], [25.093396864801022, 21.43178559489606], [23.33452377915607, 23.33452377915607], [21.43178559489606, 25.093396864801022], [19.396913325651614, 26.697560814373265], [17.242452635626314, 28.137125423685042], [14.981686491405044, 29.403215298216136], [12.628553268047964, 30.48802457287246], [10.197560814373265, 31.384865037740067], [7.70369700724488, 32.088207373123325], [5.162337346327621, 32.59371523963954], [2.589150159018885, 32.89827201319322]])
x, y = data.T

# plot our list in X,Y coordinates
plt.scatter(x, y)
plt.show()

