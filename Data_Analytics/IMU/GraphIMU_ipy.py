# %%
# ipython in vscode
print("IMU")

import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

rm_set = set()
SENSOR_DATA = "IMU.csv"
DATA_SVG = "IMU.svg"
DATA_PNG = "IMU.png"


df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
print(df.head())
# remove rows with header, drop blanks, convert data to float
df = df[df['Scaled_Acc_X_(mg)'] != 'Scaled_Acc_X_(mg)']
df = df.dropna()
df = df.astype(float)

ax = df.plot(title="IMU Data", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
fig = ax.get_figure()
fig.set_size_inches(12, 9)
fig.savefig(DATA_SVG, dpi = 100)
# fig.savefig(DATA_PNG, dpi = 100)

# %%
import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "IMU.csv"
ACC_SVG = "ACC.svg"
ACC_PNG = "ACC.png"
# Acc (mg)
print("Acc (mg)")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
# remove rows with header, drop blanks, convert data to float
df = df[df['Scaled_Acc_X_(mg)'] != 'Scaled_Acc_X_(mg)']
df = df.dropna()
df = df.astype(float)

df_acc = pd.DataFrame()
df_acc = pd.concat([df_acc, df['Scaled_Acc_X_(mg)'], df['Scaled_Acc_Y_(mg)'], df['Scaled_Acc_Z_(mg)']], axis=1)
ax = df_acc.plot(title="IMU Scaled Acc (mg)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Scaled Acc (mg)")
fig = ax.get_figure()
fig.set_size_inches(12, 9)
fig.savefig(ACC_SVG, dpi = 100)
# fig.savefig(ACC_PNG, dpi = 100)

# %%
import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "IMU.csv"
GYR_SVG = "GYR.svg"
GYR_PNG = "GYR.png"
# Gyr (DPS)
print("Gyr (DPS)")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
# remove rows with header, drop blanks, convert data to float
df = df[df['Scaled_Acc_X_(mg)'] != 'Scaled_Acc_X_(mg)']
df = df.dropna()
df = df.astype(float)

ax = df[['Gyr_X_(DPS)', 'Gyr_Y_(DPS)', 'Gyr_Z_(DPS)']].plot(title="IMU Gyr (DPS)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Gyr (DPS)")
fig = ax.get_figure()
fig.set_size_inches(12, 9)
fig.savefig(GYR_SVG, dpi = 100)
# fig.savefig(GYR_PNG, dpi = 100)

# %%
import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "IMU.csv"
MAG_SVG = "MAG.svg"
MAG_PNG = "MAG.png"
# Mag (uT)
print("Mag (uT)")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
# remove rows with header, drop blanks, convert data to float
df = df[df['Scaled_Acc_X_(mg)'] != 'Scaled_Acc_X_(mg)']
df = df.dropna()
df = df.astype(float)

ax = df[['Mag_X_(uT)', 'Mag_Y_(uT)', 'Mag_Z_(uT)']].plot(title="IMU Mag (uT)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Mag (uT)")
fig = ax.get_figure()
fig.set_size_inches(12, 9)
fig.savefig(MAG_SVG, dpi = 100)
# fig.savefig(MAG_PNG, dpi = 100)
# %%
import os  
import pandas as pd

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "IMU.csv"
TMP_SVG = "Temperature.svg"
TMP_PNG = "Temperature.png"
# Temperature (C)
print("Temperature (C)")

df = pd.read_csv(SENSOR_DATA, index_col=0, parse_dates=True)
# remove rows with header, drop blanks, convert data to float
df = df[df['Scaled_Acc_X_(mg)'] != 'Scaled_Acc_X_(mg)']
df = df.dropna()
df = df.astype(float)

ax = df['Tmp_(C)'].plot(title="IMU Temperature (C)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Temperature (C)")
fig = ax.get_figure()
fig.set_size_inches(12, 9)
fig.savefig(TMP_SVG, dpi = 100)
# fig.savefig(TMP_PNG, dpi = 100)
# %%

