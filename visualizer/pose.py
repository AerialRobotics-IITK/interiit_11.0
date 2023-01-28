import pandas as pd
import matplotlib.pyplot as plt
import sys

if len(sys.argv) < 2:
    print("Pass path to time,x,y,z,x_o,y_o,z_o estimated log")
    exit(1)

df = pd.read_csv(sys.argv[1])
print(df.head())

plt.plot(df['time'].values, df['x'].values, label="x")
plt.plot(df['time'].values, df['y'].values, label="y")
plt.plot(df['time'].values, df['z'].values, label="z")
plt.legend(loc="upper left")
plt.show()
