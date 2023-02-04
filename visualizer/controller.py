import pandas as pd
import matplotlib.pyplot as plt
import sys

if len(sys.argv) < 2:
    print("Pass path to time,e_d,e_d,e_i,roll,pitch,yaw,thrust estimated log")
    exit(1)

df = pd.read_csv(sys.argv[1])
print(df.head())

fig, ax = plt.subplots(nrows=3, ncols=3)
ax[0][0].plot(df['time'].values, df['e_p_x'].values)
ax[0][1].plot(df['time'].values, df['e_d_x'].values)
ax[0][2].plot(df['time'].values, df['e_i_x'].values)
ax[1][0].plot(df['time'].values, df['e_p_y'].values)
ax[1][1].plot(df['time'].values, df['e_d_y'].values)
ax[1][2].plot(df['time'].values, df['e_i_y'].values)
ax[2][0].plot(df['time'].values, df['e_p_z'].values)
ax[2][1].plot(df['time'].values, df['e_d_z'].values)
ax[2][2].plot(df['time'].values, df['e_i_z'].values)
ax[0][0].set_title("e_p_x")
ax[0][1].set_title("e_d_x")
ax[0][2].set_title("e_i_x")
ax[1][0].set_title("e_p_y")
ax[1][1].set_title("e_d_y")
ax[1][2].set_title("e_i_y")
ax[2][0].set_title("e_p_z")
ax[2][1].set_title("e_d_z")
ax[2][2].set_title("e_i_z")
fig.show()

pig, bx = plt.subplots(nrows=2, ncols=2)
bx[0][0].plot(df['time'].values, df['thrust'].values)
bx[0][1].plot(df['time'].values, df['roll'].values)
bx[1][0].plot(df['time'].values, df['pitch'].values)
bx[1][1].plot(df['time'].values, df['yaw'].values)
bx[0][0].set_title("thrust")
bx[0][1].set_title("roll")
bx[1][0].set_title("pitch")
bx[1][1].set_title("yaw")
pig.show()

plt.figure()
plt.plot(df['time'].values, df['x'].values, label="x")
plt.plot(df['time'].values, df['y'].values, label="y")
plt.plot(df['time'].values, df['z'].values, label="z")
plt.legend(loc="upper left")
plt.show()
