#!/usr/bin/env python
#-*- coding:utf-8 -*-	# 한글 주석을 달기 위해 사용한다.

import csv

import math

import numpy as np

from matplotlib import pyplot as plt
from sklearn.metrics import mean_squared_error
import copy
import pymap3d as pm
def RMSE(a,b):

    c = (float(a) - float(b)) ** 2

    d = np.sqrt(c)
    if d >= 100:
        c = (-float(a) - float(b)) ** 2
        d = np.sqrt(c)
    return d

def main():

    # ------------------------------
    f = open('data_kalman.csv', 'r', encoding="UTF-8")
    rdr = csv.reader(f)
    kalman = []
    for line in rdr:
        row_list = []
        for i in range(len(line)):
            num = float(line[i])
            row_list.append(num)
        kalman.append(row_list)
    kalman = np.array(kalman)
    f.close()
    print(len(kalman[0]))
    # ------------------------------
    f = open('span_0727.csv', 'r', encoding="UTF-8")
    rdr = csv.reader(f)
    span = []
    for line in rdr:
        row_list = []
        for i in range(len(line)):
            num = float(line[i])
            row_list.append(num)
        span.append(row_list)
    span = np.array(span)
    f.close()

    # ------------------------------
    f = open('vn100.csv', 'r', encoding="UTF-8")
    rdr = csv.reader(f)
    vn100 = []
    for line in rdr:
        row_list = []
        for i in range(len(line)):
            num = float(line[i])
            row_list.append(num)
        vn100.append(row_list)
    vn100 = np.array(vn100)
    f.close()

    # ------------------------------
    f = open('NED_data.csv', 'r', encoding="UTF-8")
    rdr = csv.reader(f)
    NED_data = []
    for line in rdr:
        row_list = []
        for i in range(len(line)):
            num = float(line[i])
            row_list.append(num)
        NED_data.append(row_list)
    NED_data = np.array(NED_data)
    f.close()
    # span
    plt_vn100 = []
    for i in range(len(vn100)):
        for k in range(1):
            plt_vn100.append(vn100[i])

    plt_vn100 = np.array(plt_vn100)

    span_LLH = []

    for i in range(len(span)):
        lat, lon, alt = pm.geodetic2ned(span[i][0], span[i][1], span[i][2],span[0][0],span[0][1],span[0][2])
        span_LLH.append([lat, lon, alt])
    span_LLH =np.array(span_LLH)
    print("span init : {}".format(span[0]))
    # bestpos.lat,
    # bestpos.lon,
    # bestpos.height,
    # bestxyz.x_vel,
    # bestxyz.y_vel,
    # bestxyz.z_vel,
    # inspva.longitude,
    # inspva.height,
    # inspva.north_velocity,
    # inspva.east_velocity,
    # inspva.up_velocity,
    # inspva.roll,
    # inspva.pitch,
    # yaw,
    # corrimudata.pitch_rate,
    # corrimudata.roll_rate,
    # corrimudata.yaw_rate,
    # corrimudata.lateral_acceleration,
    # corrimudata.longitudinal_acceleration,
    # corrimudata.vertical_acceleration

    tmp = [0]
    for i in range(len(plt_vn100)):
        tmp.append(0)
    tmp = np.array(tmp)
    error_imu = []
    for i in range(len(plt_vn100)):

        accel_x_error   = RMSE(vn100[i, 6], 0)
        accel_y_error   = RMSE(vn100[i, 7], 0)
        accel_z_error   = RMSE(vn100[i, 8], 0)
        angular_x_error = RMSE(vn100[i, 9], 0)
        angular_y_error = RMSE(vn100[i, 10], 0)
        angular_z_error = RMSE(vn100[i, 11], 0)
        error_imu.append([accel_x_error  ,
                        accel_y_error  ,
                        accel_z_error  ,
                        angular_x_error,
                        angular_y_error,
                        angular_z_error])

    error_imu = np.array(error_imu)
    print("accel_x_MSE std : ", np.std(error_imu[:, 0]))
    print("accel_x_MSE mean : ", np.mean(error_imu[:, 0]))

    print("accel_y_MSE std : ", np.std(error_imu[:, 1]))
    print("accel_y_MSE mean : ", np.mean(error_imu[:, 1]))

    print("accel_z_MSE std : ", np.std(error_imu[:, 2]))
    print("accel_z_MSE mean : ", np.mean(error_imu[:, 2]))

    print("angular_x_MSE std : ", np.std(error_imu[:, 3]))
    print("angular_x_MSE mean : ", np.mean(error_imu[:, 3]))

    print("angular_y_MSE std : ", np.std(error_imu[:, 4]))
    print("angular_y_MSE mean : ", np.mean(error_imu[:, 4]))

    print("angular_z_MSE std : ", np.std(error_imu[:, 5]))
    print("angular_z_MSE mean : ", np.mean(error_imu[:, 5]))



    error =[]
    for i in range(len(plt_vn100)):
        try:
            vn300_vs_kalman_MSE = RMSE(kalman[i, 8], span[i, 13])
            GPSyaw_vs_kalman_MSE = 0

            pitch_MSE = RMSE(kalman[i, 7], span[i, 12])
            roll_MSE = RMSE(kalman[i, 6], span[i, 11])

            vn_MSE = RMSE(kalman[i, 3], span[i, 8])
            ve_MSE = RMSE(kalman[i, 4], span[i, 9])
            vd_MSE = RMSE(kalman[i, 5], span[i, 10])

            error.append([vn300_vs_kalman_MSE,
                          GPSyaw_vs_kalman_MSE,
                          pitch_MSE,
                          roll_MSE,
                          vn_MSE,
                          ve_MSE,
                          vd_MSE])
        except:
            pass


    error = np.array(error)

    print("vn300_vs_kalman_MSE std : ",np.std(error[:,0]))
    print("vn300_vs_kalman_MSE mean : ",np.mean(error[:,0]))

    print("GPSyaw_vs_kalman_MSE std : ", np.std(error[:, 1]))
    print("GPSyaw_vs_kalman_MSE mean : ", np.mean(error[:, 1]))

    print("pitch_MSE std : ", np.std(error[:, 2]))
    print("pitch_MSE mean : ", np.mean(error[:, 2]))

    print("roll_MSE std : ", np.std(error[:, 3]))
    print("roll_MSE mean : ", np.mean(error[:, 3]))

    print("vn_MSE std : ", np.std(error[:, 4]))
    print("vn_MSE mean : ", np.mean(error[:, 4]))

    print("ve_MSE std : ", np.std(error[:, 5]))
    print("ve_MSE mean : ", np.mean(error[:, 5]))

    print("vd_MSE std : ", np.std(error[:, 6]))
    print("vd_MSE mean : ", np.mean(error[:, 6]))

    #



    print(span[0][13],span[0][12],span[0][11])
    plt.figure(1)
    plt.subplot(311)
    plt.plot(kalman[:, 8], 'r-',label='UKF')
    plt.plot(span[:, 13], 'g-', label='SPAN')
    plt.ylabel('yaw(deg)')
    plt.title('yaw')

    plt.legend()

    plt.subplot(312)
    plt.plot(kalman[:, 7], 'r-', )
    plt.plot(span[:, 12], 'g-', label='SPAN')
    plt.ylabel('pitch(deg)')
    plt.title('pitch')

    plt.subplot(313)
    plt.plot(kalman[:, 6], 'r-')
    plt.plot(span[:, 11], 'g-', label='SPAN')
    plt.ylabel('roll(deg)')
    plt.title('roll')

    plt.figure(2)

    plt.subplot(311)

    plt.plot(span[:, 4], 'g-', markersize = 1, label='SPAN')
    plt.plot(kalman[:, 3], 'r-',markersize = 1,  label='UKF')
    # plt.plot(span[:, 4], 'ko', markersize = 1)
    # plt.plot(plt_vn100[:, 3], 'b-', label='GPS_Velocity')
    plt.ylabel('v_n(m/s)')
    plt.title('v_n')
    plt.legend()

    plt.subplot(312)

    plt.plot(-span[:, 3], 'g-',markersize = 1)
    plt.plot(kalman[:, 4], 'r-', markersize = 1)
    # plt.plot(span[:, 3], 'ko', markersize=1)
    # plt.plot(plt_vn100[:, 4], 'b-')
    plt.ylabel('v_e(m/s)')
    plt.title('v_e')

    plt.subplot(313)

    plt.plot(span[:, 5], 'g-',markersize = 1)
    plt.plot(kalman[:, 5], 'r-',markersize = 1)
    # plt.plot(-span[:, 5], 'ko', markersize=1)
    # plt.plot(plt_vn100[:, 5], 'b-')
    plt.ylabel('v_d(m/s)')
    plt.title('v_d')

    plt.figure(3)

    plt.subplot(311)

    plt.plot(vn100[:, 6], 'r-', label='vn100_accel')
    plt.title('accel_x')
    plt.legend()

    plt.subplot(312)

    plt.plot(vn100[:, 7], 'r-')
    plt.title('accel_y')

    plt.subplot(313)

    plt.plot(vn100[:, 8], 'r-')
    plt.title('accel_z')

    plt.figure(4)

    plt.subplot(311)

    plt.plot(vn100[:, 9], 'r-', label='vn100_gyro')
    plt.title('angular_x')
    plt.legend()

    plt.subplot(312)

    plt.plot(vn100[:, 10], 'r-')
    plt.title('angular_y')

    plt.subplot(313)

    plt.plot(vn100[:, 11], 'r-')
    plt.title('angular_z')

    plt.figure(5)

    plt.subplot(111)
    plt.plot(vn100[:, 12], 'ro',markersize = 1,  label='UKF_dt')
    plt.plot(vn100[:, 13], 'bo', markersize = 1,label='GPS_dt')
    plt.title('dt')
    plt.legend()

    plt.figure(6)

    plt.subplot(111)

    # n e
    plt.plot(span_LLH[:, 0], span_LLH[:, 1], "ob", markersize=3, label='SPAN')

    # n e


    plt.plot(NED_data[:, 1], NED_data[:, 0], "or", markersize=3,label='UKF')

    plt.plot(NED_data[:, 2], NED_data[:, 3], "og", markersize=4,label='RTK')
    x_cent = (sum(NED_data[:, 1]) / len(NED_data[:, 0]) + sum(NED_data[:, 2]) / len(NED_data[:, 0])) /2
    y_cent = (sum(NED_data[:, 0]) / len(NED_data[:, 0]) + sum(NED_data[:, 3]) / len(NED_data[:, 0])) /2
    plt.xlim(x_cent-180, x_cent+180)
    plt.ylim(y_cent - 180, y_cent + 180)
    plt.title('SPAN vs UKF')
    plt.legend()
    #
    error = np.array(error)

    plt.figure(7)
    plt.subplot(311)
    plt.plot(error[:, 0], 'b-', label='yaw error')

    plt.legend()
    plt.subplot(312)
    plt.plot(error[:, 2], 'b-', label='pitch error')
    plt.legend()
    plt.subplot(313)
    plt.plot(error[:, 3], 'b-', label='roll error')
    plt.legend()

    plt.figure(8)

    plt.subplot(311)
    plt.plot(error[:, 4], 'b-', label='vn error')
    plt.legend()
    plt.subplot(312)
    plt.plot(error[:, 5], 'b-', label='ve error')
    plt.legend()
    plt.subplot(313)
    plt.plot(error[:, 6], 'b-', label='vd error')
    plt.legend()

    plt.show()





if __name__ == '__main__':
    main()
