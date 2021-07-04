# %%
# ipython in vscode
print("plot")

import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "RTD.csv"
DATA_SVG = "RTD.svg"
DATA_PNG = "RTD.png"


df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
df.head()
# remove rows with header, drop blanks, convert data to float
df = df[df['RTD1_(C)'] != 'RTD1_(C)']
df = df.dropna()
df = df.astype(float)

ax = df.plot(title="RTD temperature (°C)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Temperature (°C)")
fig = ax.get_figure()
fig.set_size_inches(12, 9)
fig.savefig(DATA_SVG, dpi = 100)
fig.savefig(DATA_PNG, dpi = 100)

# %%
import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "RTD.csv"
# RTD1
print("RTD1")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
ax = df['RTD1_(C)'].plot(title="RTD1 temperature (°C)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Temperature (°C)")

# %%
import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "RTD.csv"
# RTD2
print("RTD2")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
ax = df['RTD2_(C)'].plot(title="RTD2 temperature (°C)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Temperature (°C)")

# %%
import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "RTD.csv"
# RTD3
print("RTD3")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
ax = df['RTD3_(C)'].plot(title="RTD3 temperature (°C)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Temperature (°C)")
# %%
import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "RTD.csv"
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

SENSOR_DATA = "RTD.csv"
# area
print("area")

import matplotlib.pyplot as plt

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
df.head()
fig, axs = plt.subplots(figsize=(12, 4))
df.plot.area(ax=axs, subplots=True, title="RTD temperature (°C) subplot")
axs.set_ylabel("Temperature (°C)")
# %%

