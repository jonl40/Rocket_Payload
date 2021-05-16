# %%
# ipython in vscode
print("plot")

import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "RTD1.csv"
DATA_SVG = "RTD1.svg"
DATA_PNG = "RTD1.png"


df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
df.head()
ax = df.plot(title="RTD temperature (°C)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Temperature (°C)")
fig = ax.get_figure()
fig.set_size_inches(12, 9)
fig.savefig(DATA_SVG, dpi = 100)
fig.savefig(DATA_PNG, dpi = 100)

# %%

# RTD1
print("RTD1")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
ax = df['RTD1_(C)'].plot(title="RTD1 temperature (°C)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Temperature (°C)")

# %%

# RTD2
print("RTD2")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
ax = df['RTD2_(C)'].plot(title="RTD2 temperature (°C)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Temperature (°C)")

# %%

# RTD3
print("RTD3")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
ax = df['RTD3_(C)'].plot(title="RTD3 temperature (°C)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Temperature (°C)")
# %%

# subplot
print("subplot")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
df.head()
axs = df.plot(figsize=(12, 4), subplots=True, title="RTD temperature (°C) subplot")
# %%

# area
print("area")

import matplotlib.pyplot as plt

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
df.head()
fig, axs = plt.subplots(figsize=(12, 4))
df.plot.area(ax=axs, subplots=True, title="RTD temperature (°C) subplot")
axs.set_ylabel("Temperature (°C)")
# %%

