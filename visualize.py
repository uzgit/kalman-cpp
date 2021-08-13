#!/usr/bin/python3

import pandas
import matplotlib.pyplot as plt

data = pandas.read_csv("output.csv")

data["jerk"] = data["acceleration"].diff()

print(data.head())

figure = plt.figure()

plt.scatter(data["Time"], data["Measurement"], s=3, color="blue", label="Measurements")
plt.plot(data["Time"], data["Estimate"], color="red", label="Estimate")


plt.xlabel("Time")
plt.ylabel("Distance")

plt.title("Sample Kalman filter performance for transformed pose with ambiguous orientation.")

plt.legend()

plt.figure()
plt.plot(data["Time"], data["velocity"], color="red", label="Velocity")
plt.plot(data["Time"], data["acceleration"], color="blue", label="Acceleration")
plt.plot(data["Time"], data["jerk"], color="pink", label="Jerk")
plt.xlabel("Time")
plt.legend()

plt.show()
