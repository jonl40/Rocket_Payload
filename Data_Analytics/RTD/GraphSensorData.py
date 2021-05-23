import pandas as pd
import matplotlib.pyplot as plt
import os  
from collections import deque

CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))
os.chdir(CUR_FOLDER)

SENSOR_DATA = "RTD_sensor.csv"

#df = pd.read_csv("RTD.csv", index_col=0, parse_dates=True)
df = pd.read_csv("RTD.csv")
df.head()
df.plot(title="RTD temperature (°C)")


data_dq = deque()
for col in df.columns:
    data_dq.append(col)
time_data = data_dq.popleft()
print(time_data)
print(data_dq)


'''
data_cols = df.columns
data_cols = data_cols[1:]
time = df.columns[0]
print(f"time: {time}")
print(f"data_cols: {data_cols}")
'''


fig = plt.figure() 
ax = fig.add_subplot(111) 
plt.title('RTD Sensor Data')
plt.xlabel("Time")
plt.ylabel("Temperature (c)")
# beautify the x-labels
plt.gcf().autofmt_xdate()
# plot RTD data 
for col in data_dq:
    plt.plot(df[time_data], df[col], label=col)
#grid 
plt.grid()
#legend 
plt.legend() 
fig.set_size_inches(12, 9)
fig.savefig("RTD.svg", dpi = 100)
fig.savefig("RTD.png", dpi = 100)
plt.savefig("RTD_test.png", bbox_inches='tight')
plt.show()
