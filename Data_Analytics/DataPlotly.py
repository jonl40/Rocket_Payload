import pandas as pd
import matplotlib.pyplot as plt 
import plotly.express as px
import re
import os   
import sys 
import time 

IMU_CSV = "IMU.csv"
RTD_CSV = "RTD.csv"

TIME = "DateTime"
SCALED_ACC_RE = r'Scaled_Acc_\w_\(mg\)'
GYR_RE = r'Gyr_\w_\(DPS\)'
MAG_RE = r'Mag_\w_\(uT\)'
RTD_RE = r'RTD\d_\(C\)'

class PayloadPlotter:
    def __init__(self, imu_file, rtd_file, datetime):
        self.imu_file = imu_file
        self.rtd_file = rtd_file
        self.datetime = datetime

        if imu_file: 
            self.df_imu = pd.read_csv(self.imu_file, engine='c', encoding='unicode_escape')
            
        if rtd_file:
            self.df_rtd = pd.read_csv(self.rtd_file, engine='c', encoding='unicode_escape')
        
        self.preprocess_df(datetime)


    def preprocess_df(self, date):
        if self.imu_file: 
            # datetime to num, remove rows with header, drop blanks
            self.df_imu = self.df_imu[self.df_imu[date] != date]
            self.df_imu[date] = self.df_imu[date].str.strip()
            self.df_imu[date] = pd.to_datetime(self.df_imu[date], errors='coerce') 
            self.df_imu[self.df_imu.columns[1:]] = self.df_imu[self.df_imu.columns[1:]].apply(lambda x: pd.to_numeric(x, errors='coerce')).dropna()

            # convert str to float 
            for col in self.df_imu.columns[1:]:
                #self.df_imu[col] = self.df_imu[col].str.strip()
                self.df_imu[col] = self.df_imu[col].astype(float)

            print("\n==== IMU ===\n")
            print(self.df_imu.head())
        
        if self.rtd_file: 
            # datetime to num, remove rows with header, drop blanks
            self.df_rtd = self.df_rtd[self.df_rtd[date] != date]
            self.df_rtd[date] = self.df_rtd[date].str.strip()
            self.df_rtd[date] = pd.to_datetime(self.df_rtd[date], errors='coerce') 
            self.df_rtd[self.df_rtd.columns[1:]] = self.df_rtd[self.df_rtd.columns[1:]].apply(lambda x: pd.to_numeric(x, errors='coerce')).dropna()

            # convert str to float 
            for col in self.df_rtd.columns[1:]:
                #self.df_rtd[col] = self.df_rtd[col].str.strip()
                self.df_rtd[col] = self.df_rtd[col].astype('float16')

            print("\n==== RTD ===\n")
            print(self.df_rtd.head())


    def imu_plot_all(self, title_, xaxis, yaxis, pic_name=None):
        if self.imu_file:
            tmp = self.df_imu.columns[1:]
            fig = px.line(self.df_imu, x=xaxis, y=tmp, 
                labels={"variable": "Sensor", "value": yaxis}, 
                title=title_, template="plotly_dark", markers=True)
            
            fig.show()
            if pic_name:
                fig.write_html(pic_name)


    def sensor_plot(self, device, sensor_regex, title_, xaxis, yaxis, pic_name=None):
        if self.imu_file and device == "imu": 
            df = self.df_imu
            # print(id(df) == id(self.df_imu))
        elif self.rtd_file and device == "rtd":
            df = self.df_rtd
        elif device == "imu" and not self.imu_file:
            print("Error: No imu file was provided during object initialization!")
            sys.exit(0)
        elif device == "rtd" and not self.df_rtd:
            print("Error: No RTD file was provided during object initialization!")
            sys.exit(0)
        elif device != "rtd" and device != "imu":
            print("device must be 'rtd' or 'imu' !")
            sys.exit(0)

        tmp = []
        for column in df:
            m = re.match(sensor_regex, column)
            if m is not None:
                tmp.append(m.group(0))
            fig = px.line(df, x=xaxis, y=tmp, 
                labels={"variable": "Sensor", "value": yaxis}, 
                title=title_, template="plotly_dark", markers=True)

        fig.show()
        if pic_name:
            fig.write_html(pic_name)


def main():
    start = time.time()
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    grapher = PayloadPlotter(IMU_CSV, RTD_CSV, TIME)

    grapher.imu_plot_all("IMU sensors", TIME, "", "IMU_Sensors.html")
    grapher.sensor_plot("imu", SCALED_ACC_RE, "IMU Scaled Acc (mg)", TIME, "Scaled Acc (mg)", "ACC.html")
    grapher.sensor_plot("imu", GYR_RE, "IMU Gyr (DPS)", TIME, "Gyr (DPS)", "GYR.html")
    grapher.sensor_plot("imu", MAG_RE, "IMU Mag (uT)", TIME, "Mag (uT)", "MAG.html")
    grapher.sensor_plot("rtd", RTD_RE, "RTD temperature (°C)", TIME, "Temperature (°C)", "RTD.html")

    end = time.time()
    tot_time = end - start 
    print("{:.3f} s".format(tot_time))


main()