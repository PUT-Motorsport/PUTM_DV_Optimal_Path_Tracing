import pandas as pd
from matplotlib import pyplot as plt

# Load the csv file into a pandas DataFrame
df = pd.read_csv('path.csv')

# Extract the x and y columns as separate Series
x = df.iloc[:, 0]
y = df.iloc[:, 1]

plt.plot(x, y)
plt.show()