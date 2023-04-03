import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

# Load the csv file into a pandas DataFrame
df = pd.read_csv('stanley.csv')

# Extract the x and y columns as separate Series
x = df.iloc[:, 0]
y = df.iloc[:, 1]

df = pd.read_csv("track.csv")

x_track = df.iloc[:, 0]
y_track = df.iloc[:, 1]

plt.plot(x_track, y_track)
plt.plot(x, y)
plt.show()

plt.margins(216, 5)
plt.show()