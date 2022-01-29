import pandas as pd
import matplotlib.pyplot as plt 
import re
import os   
import sys 

IMU_CSV = "IMU.csv"
RTD_CSV = "RTD.csv"

TIME = "DateTime"
SCALED_ACC_RE = r'Scaled_Acc_\w_\(mg\)'
GYR_RE = r'Gyr_\w_\(DPS\)'
MAG_RE = r'Mag_\w_\(uT\)'
RTD_RE = r'RTD\d_\(C\)'

HEIGHT = 19 
WIDTH = 10
DPI = 200


class PayloadPlotter:
    def __init__(self, imu_file, rtd_file, datetime):
        self.imu_file = imu_file
        self.rtd_file = rtd_file
        self.datetime = datetime

        if imu_file: 
            self.df_imu = pd.read_csv(self.imu_file)
            
        if rtd_file:
            self.df_rtd = pd.read_csv(self.rtd_file)
        
        self.preprocess_df(datetime)


    def preprocess_df(self, date):
        if self.imu_file: 
            # datetime to num, remove rows with header, drop blanks
            self.df_imu[date] = pd.to_datetime(self.df_imu[date]) 
            self.df_imu = self.df_imu[self.df_imu['Scaled_Acc_X_(mg)'] != 'Scaled_Acc_X_(mg)']
            self.df_imu = self.df_imu.dropna()
            print("\n==== IMU ===\n")
            print(self.df_imu.head())
        
        if self.rtd_file: 
            # datetime to num, remove rows with header, drop blanks
            self.df_rtd[date] = pd.to_datetime(self.df_rtd[date]) 
            self.df_rtd = self.df_rtd[self.df_rtd['RTD1_(C)'] != 'RTD1_(C)']
            self.df_rtd = self.df_rtd.dropna()
            print("\n==== RTD ===\n")
            print(self.df_rtd.head())


    def save_pic(self, graph, name):
        if name:
            figure = graph.gcf()
            figure.set_size_inches(HEIGHT, WIDTH)
            graph.savefig(name, dpi=DPI)


    def imu_plot_all(self, title, xaxis, yaxis, pic_name=None):
        if self.imu_file:
            imu_cols = self.df_imu.columns
            for i in range(1, len(imu_cols)):
                plt.plot(self.df_imu[xaxis], self.df_imu[imu_cols[i]], '-o', label=imu_cols[i])
            plt.title(title)
            plt.xlabel(xaxis)
            plt.ylabel(yaxis)
            plt.legend(loc="best")
            plt.grid()
            self.save_pic(plt, pic_name)
            plt.show()


    def sensor_plot(self, device, sensor_regex, title, xaxis, yaxis, pic_name=None):
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

        for column in df:
            m = re.match(sensor_regex, column)
            if m is not None:
                plt.plot(df[xaxis], df[m.group(0)], '-o', label=m.group(0))

        plt.title(title)
        plt.xlabel(xaxis)
        plt.ylabel(yaxis)
        plt.legend(loc="best")
        plt.grid()
        self.save_pic(plt, pic_name)
        plt.show()



def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    grapher = PayloadPlotter(IMU_CSV, RTD_CSV, TIME)

    grapher.imu_plot_all("IMU sensors", TIME, "", "IMU_Sensors.png")
    grapher.sensor_plot("imu", SCALED_ACC_RE, "IMU Scaled Acc (mg)", TIME, "Scaled Acc (mg)", "ACC.png")
    grapher.sensor_plot("imu", GYR_RE, "IMU Gyr (DPS)", TIME, "Gyr (DPS)", "GYR.png")
    grapher.sensor_plot("imu", MAG_RE, "IMU Mag (uT)", TIME, "Mag (uT)", "MAG.png")
    grapher.sensor_plot("rtd", RTD_RE, "RTD temperature (°C)", TIME, "Temperature (°C)", "RTD.png")


main()