# %%
# ipython in vscode
print("plot")

import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "IMU.csv"
DATA_SVG = "IMU.svg"
DATA_PNG = "IMU.png"


df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
print(df.head())
ax = df.plot(title="IMU Data", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
fig = ax.get_figure()
fig.set_size_inches(12, 9)
fig.savefig(DATA_SVG, dpi = 100)
fig.savefig(DATA_PNG, dpi = 100)

# %%
import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "IMU.csv"
# Acc
print("Acc")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
print(df.head())
df_acc = pd.DataFrame()
df_acc = pd.concat([df_acc, df['Scaled_Acc_X_(mg)'], df['Scaled_Acc_Y_(mg)'], df['Scaled_Acc_Z_(mg)']], axis=1)
ax = df_acc.plot(title="IMU Scaled Acc (mg)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Scaled Acc (mg)")

# %%
import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "IMU.csv"
# RTD2
print("RTD2")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
print(df.head())
ax = df['RTD2_(C)'].plot(title="RTD2 temperature (°C)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Temperature (°C)")

# %%
import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "IMU.csv"
# RTD3
print("RTD3")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
print(df.head())
ax = df[['Gyr_X_(DPS)', 'Gyr_Y_(DPS)', 'Gyr_Z_(DPS)']].plot(title="IMU Gyr (DPS)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Gyr (DPS)")
# %%
import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "IMU.csv"
# subplot
print("subplot")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
df.head()
axs = df.plot(figsize=(12, 4), subplots=True, title="RTD temperature (°C) subplot")
# %%
import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "IMU.csv"
# area
print("area")

import matplotlib.pyplot as plt

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
df.head()
fig, axs = plt.subplots(figsize=(12, 4))
df.plot.area(ax=axs, subplots=True, title="RTD temperature (°C) subplot")
axs.set_ylabel("Temperature (°C)")
# %%

