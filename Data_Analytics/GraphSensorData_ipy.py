# %%
# ipython in vscode

import os  
import pandas as pd
#import matplotlib.pyplot as plt, mpld3

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "RTD_sensor.csv"


df = pd.read_csv("RTD.csv", index_col=0, parse_dates=True)
df.head()
ax = df.plot(title="RTD temperature (°C)", grid=True, marker='.', markersize=10)
ax.set_xlabel("DateTime")
ax.set_ylabel("Temperature (°C)")
#mpld3.save_html(ax, 'f.html') #save to html here
fig = ax.get_figure()
fig.set_size_inches(12, 9)
fig.savefig("RTD.svg", dpi = 100)
fig.savefig("RTD.png", dpi = 100)
#fig.save_html(ax, 'f.html')

# %%
