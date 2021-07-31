#!/usr/bin/python3

import pandas
import matplotlib.pyplot as plt

data = pandas.read_csv("output.csv")

print(data.head())

figure = plt.figure()

plt.scatter(data["Time"], data["Measurement"], s=3, color="blue", label="Measurements")
plt.plot(data["Time"], data["Estimate"], color="red", label="Estimate")

plt.xlabel("Time")
plt.ylabel("Distance")

plt.title("Sample Kalman filter performance for transformed pose with ambiguous orientation.")

plt.legend()

plt.show()
