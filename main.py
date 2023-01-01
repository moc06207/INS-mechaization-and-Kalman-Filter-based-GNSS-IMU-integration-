#!/usr/bin/env python
#-*- coding:utf-8 -*-	# 한글 주석을 달기 위해 사용한다.
import rospy
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, PointCloud2
import time
from multiprocessing import Value, Process, Queue
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import copy
import numpy as np
import pymap3d as pm
import matplotlib.pyplot as plt
show_animation = True
from std_msgs.msg import Float32MultiArray
import csv
from geometry_msgs.msg import TwistWithCovarianceStamped


# IMU 수신 콜백함수
def chatterCallback(data):

    # 1. 가속도, 각속도, 쿼터니안
    current_accel_x.value, current_accel_y.value, current_accel_z.value = round(data.linear_acceleration.x,2), round(data.linear_acceleration.y,2), round(data.linear_acceleration.z,2)

    current_vel_x.value, current_vel_y.value,current_vel_z.value = -round(data.angular_velocity.x,5), round(data.angular_velocity.y,5), -round(data.angular_velocity.z,5)

    current_quat_x.value, current_quat_y.value, current_quat_z.value, current_quat_w.value = round(data.orientation.x,2), round(data.orientation.y,2), round(data.orientation.z,2), round(data.orientation.w,2)

    # 쿼터니안을 yaw, pitch, roll변환을 위해 배열 선언
    quat_list = [current_quat_x.value, current_quat_y.value,current_quat_z.value, current_quat_w.value]

    # euler_from_quaternion 이라는 라이브러리를 이용하요 ypr 계산
    # 단위는 라디안
    roll, pitch, yaw = euler_from_quaternion(quat_list)
    current_roll.value = round(roll, 4)
    current_pitch.value = round(pitch, 4)
    current_yaw.value = round(yaw, 4)

    # 라디안을 각도로 변환하기 위해 라이브러리 이용
    yaw = math.degrees(yaw)

    # CTC, CNT설명은 무시
    IMU_CTC.value = 1
    IMU_CNT.value += 1
    if IMU_CNT.value > 255:
        IMU_CNT.value = 0

# GPS 수신 콜백함수
def GPSCallback(data):

    current_lat.value, current_lon.value, current_alt.value = data.latitude, data.longitude, data.altitude

    GPS_CTC.value = 1
    GPS_CNT.value += 1
    if GPS_CNT.value > 255:
        GPS_CNT.value = 0

# GPS/IMU Integration을 칼만필터로 수행하는 메인함수!
def vn100_Kalman_filter(current_lat, current_lon, current_alt, current_accel_x,
    current_accel_y, current_accel_z,current_vel_x, current_vel_y,current_vel_z,current_yaw,
                  current_roll,current_pitch,GPS_CNT,kalman_yaw,GPS_CTC,
                        kalman_pitch, kalman_roll, kalman_lat,
                        kalman_lon, kalman_alt, kalman_NED_N, kalman_NED_E, kalman_NED_D,
                        kalman_ENU_E, kalman_ENU_N, kalman_ENU_U,
                        GPS_NED_N, GPS_NED_E, GPS_NED_D, GPS_ENU_E, GPS_ENU_N, GPS_ENU_U,IMU_CNT,GPS_fix_velocity_x,GPS_fix_velocity_y,GPS_fix_velocity_z):


    # GPS 센서 정보를 애니메이션으로 출력할 빈 배열
    GPS_comp_plot = np.empty((0, 2), float)

    # 칼만필터 정보를 애니메이션으로 출력할 빈 배열
    Kalman_comp_plot= np.empty((0, 2), float)

    # 각도와 라디안을 사전에 정의, 라이브러리에서 출력되는 값이 라디안일수도, 각도일 수도 있기 때문에
    # 이를 적절히 변환하기 위해 사용
    degree = math.pi / 180
    radian = 180 / math.pi

    # 무한루프
    while True:

        # CTC가 0이 아니라는 뜻은, GPS값이 들어왔다는 뜻
        # 다시 말해 CTC가 0이면 GPS가 들어오지 않았다는 것
        if GPS_CTC.value != 0:
            print("GPS/INS Integration Start!")

            # 칼만필터에서 배웠던 Q
            # Q는 시스템에 대한 노이즈를 의미한다
            # 시스템은 뭐다? 우리가 수식이나 모델링을 통해 최종 결과값 예를 들어 위치, 속도 등등이 되는 것들을 출력하는 것
            # Q가 크다는 것은 시스템 노이즈가 크다는 것
            # 즉, 시스템 노이즈가 크기 떄문에 센서값을 더 믿어야겠지?
            # 받대로 시스템 노이즈 Q 를 작게한다는 것은
            # 시스템을 그만큼 신뢰한다는 것

            # Q를 선언하기 위해 이중 배열 선
            Q = [[0 for j in range(15)] for i in range(15)]

            # Q는 대각행렬!! 이다.
            # 0~2는 위치
            # 3~5 속도
            # 6~8 ypr
            # 9~11 가속도
            # 12~14 각속도
            Q[0][0] = 10000.0  # LLH언
            Q[1][1] = 10000.0
            Q[2][2] = 10000.0
            Q[3][3] = 10  # v_ned
            Q[4][4] = 10
            Q[5][5] = 1000
            Q[6][6] = 0.01 # rpy
            Q[7][7] = 0.01
            Q[8][8] = 1
            Q[9][9] = 10  # accel
            Q[10][10] = 10
            Q[11][11] = 10
            Q[12][12] = 10  # gyro
            Q[13][13] = 10
            Q[14][14] = 10

            # 사실 파이썬은 배열이라는 것이 존재하지 않는다.. 리스트만 있을뿐
            # 그런데, np.array()를 이용하면 c에서 사용하는 배열을 파이썬에서도 사용할 수 있다.
            # 하이엔드 파이썬 개발자들은 기본적으로 numpy 라이브러리에서 np.array()를 이용하여 개발을 하니 알아두기
            # c에서 사용하는 배열과 문법이 거의 동일함
            Q = np.array(Q) * 10

            # P는 시스템의 공분산(covariance를 의미)
            P = [[0 for j in range(15)] for i in range(15)]

            # 초기 P값도 실험적으로 정의
            for i in range(15):
                P[i][i] = 0.01

            P = np.array(P)

            # R은 센서 노이즈
            # 앞에 Q랑 R은 15x15인데, R은 6x6 이다
            # 우리가 measurment update시 이용하는 센서는 gps가
            # gps는 위치와 속도만 나오지?
            # 이는 위치와 속도에 대해서만 update를 할 수 있다는 의미다.
            # R은 센서 노이즈를 위미하니까
            # 센서를 통해 취득할 수 있는 위치와 속도에 대해서만 신경쓰면 된다
            # 따라서, R는 6x6이다
            R = [[0 for j in range(6)] for i in range(6)]

            for i in range(6):
                R[i][i] = 2.0

            R = np.array(R)

            # H는 update 시 x와 Q에서 신경쓰고 싶은 부분만 정의하는 행렬이다
            # 그래서 15x15중 6x6에만 1이 들어가있다.
            # 나머지는 0이니, 계산 시 모두 0이 되어, 위치랑 속도만 계산하겠지?
            H = [[0 for j in range(15)] for i in range(6)]

            for i in range(6):
                H[i][i] = 1

            H = np.array(H)

            # 시작위치 사전 정의
            start_lat,start_lon,start_alt = current_lat.value, current_lon.value, current_alt.value

            # 현재 시간을 기억하는 변수 선언
            time0 = time.time()
            print('Try Attitude Initialization')



            # 여기서 부터 조금 헷갈릴 수 있다.
            # 자세한 이론은 세미나 때 설명예정
            # 우선 X 가 위치, 속도, 자세 등등에 대한 값인데
            # dX는 이에대한 변위값이다.
            # 헷갈리겠지만, 우리는 dX를 칼만필터를 통해 구하는 것이 목표이다
            # 이후 dX를 X에 더해 새로운 X를 구하는 식이다.
            dX0 = np.array([[3.1249998144744495e-09], [2.4999999848063226e-08], [-0.13608],
                            [0.0125625], [0.011875000000000002], [0.01665625],
                            [0.0007], [-0.001], [0.0007], [0.0194921875],
                            [0.3706640625], [0.02261718750000026], [0.0005827734375], [0.0006302734375],
                            [0.00054640625]])


            print('Success Attitude Initialization')

            N, E, D = pm.geodetic2ned(current_lat.value, current_lon.value, current_alt.value, start_lat,
                                      start_lon, start_alt)

            init_yaw = math.atan2(E, N) * radian  # atan2 덕분에 사분면 상관없이 yaw 나옴

            start_time = time.time()  # dt산출을 위해서!
            X0 = np.array([current_lat.value, current_lon.value, current_alt.value,
                           GPS_fix_velocity_y.value, GPS_fix_velocity_x.value, GPS_fix_velocity_z.value,
                           1.143855001, -1.620321488,
                           init_yaw])  # set initial state  RTK 와 IMU 를 이용했음

            print('start lat : ', current_lat.value)
            print('start lon : ', current_lon.value)
            print('start alt : ', current_alt.value)
            print('start north velocity : ', GPS_fix_velocity_y.value)
            print('start east velocity : ', GPS_fix_velocity_x.value)
            print('start down velocity : ', GPS_fix_velocity_z.value)
            print('start yaw : ', init_yaw)

            dX0 = np.array(dX0)

            ecc = 0.0818192  # earth's eccentricity
            R0 = 6378137.0  # earth's mean radius (m)
            Ome = 7.2921151467e-5  # earth's rotational rate (rad/s)
            g = 9.81  # gravity
            ecc_2 = ecc ** 2

            X = X0
            dX = copy.deepcopy(dX0)

            prev_N, prev_E, prev_D = pm.geodetic2ned(current_lat.value, current_lon.value, current_alt.value,
                                                     start_lat, start_lon, start_alt)
            G_CNT = GPS_CNT.value
            I_CNT = IMU_CNT.value
            G_CNT_emergency = GPS_CNT.value
            imu_cnt = IMU_CNT.value

            print("Start Kalman!")

            f = open("data_kalman.csv", "w")
            writer = csv.writer(f)

            f = open("vn100.csv", "w")
            vn100_writer = csv.writer(f)

            f = open("NED_data.csv", "w")
            NED_data = csv.writer(f)
            GPS_start_time = time.time()  # i-1 시간 다시 갱신
            GPS_dt = 0
            GPS_yaw = init_yaw

            cur_lat, cur_lon, cur_alt = current_lat.value, current_lon.value, current_alt.value
            GPS_vn, GPS_ve, GPS_vd = GPS_fix_velocity_y.value, GPS_fix_velocity_x.value, -GPS_fix_velocity_z.value
            cur_accel_x, cur_accel_y, cur_accel_z = current_accel_x.value, current_accel_y.value, current_accel_z.value
            cur_angluar_x, cur_angluar_y, cur_angluar_z = current_vel_x.value, current_vel_y.value, current_vel_z.value

            while True:

                dt = time.time() - start_time
                start_time = time.time()  # i-1 시간 다시 갱신

                while True:
                    GPS_Integration = 0
                    if GPS_CNT.value != G_CNT:

                        cur_lat, cur_lon, cur_alt = current_lat.value, current_lon.value, current_alt.value
                        GPS_vn, GPS_ve, GPS_vd = GPS_fix_velocity_y.value, GPS_fix_velocity_x.value, -GPS_fix_velocity_z.value
                        cur_accel_x, cur_accel_y, cur_accel_z = current_accel_x.value, current_accel_y.value, current_accel_z.value
                        cur_angluar_x, cur_angluar_y, cur_angluar_z = current_vel_x.value, current_vel_y.value, current_vel_z.value
                        G_CNT = GPS_CNT.value
                        imu_cnt = IMU_CNT.value
                        GPS_Integration = 1
                        break
                    else:
                        pass
                    if IMU_CNT.value != imu_cnt:

                        GPS_vn, GPS_ve, GPS_vd = GPS_fix_velocity_y.value, GPS_fix_velocity_x.value, -GPS_fix_velocity_z.value
                        cur_accel_x, cur_accel_y, cur_accel_z = current_accel_x.value, current_accel_y.value, current_accel_z.value
                        cur_angluar_x, cur_angluar_y, cur_angluar_z = current_vel_x.value, current_vel_y.value, current_vel_z.value
                        imu_cnt = IMU_CNT.value
                        break
                    else:
                        pass

                # state update with X
                lat, lon, h = X[0], X[1], X[2]
                v_n, v_e, v_d = X[3], X[4], X[5]
                roll, pitch, yaw = X[6], X[7], X[8]

                # -- ros communication --
                kalman_roll.value, kalman_pitch.value, kalman_yaw.value = float(X[6]), float(X[7]), GPS_yaw
                kalman_lat.value, kalman_lon.value, kalman_alt.value = float(X[0]), float(X[1]), float(X[2])


                #---------------------------
                #---------------------------
                #---------------------------
                # INS Error Model
                # ---------------------------
                # ---------------------------
                # ---------------------------

                # earth model
                Rm = R0 * (1 - ecc_2) / (1 - ecc_2 * (math.sin(lat * degree)) ** 2) ** 1.5
                Rt = R0 / (1 - ecc_2 * (math.sin(lat * degree)) ** 2) ** 0.5
                Rmm = (3 * R0 * (1 - ecc_2) * ecc_2 * math.sin(lat * degree)) * math.cos(lat * degree) / (
                        (1 - ecc_2 * (math.sin(lat * degree)) ** 2) ** 2.5)

                Rtt = R0 * ecc_2 * math.sin(lat * degree) * math.cos(lat * degree) / ((1 - ecc_2
                                                                                       * (math.sin(
                            lat * degree)) ** 2) ** 1.5)

                # only gyro measurement(sensor measurment) # 순수 센서 angular vel
                w_ibb = np.array([cur_angluar_x, cur_angluar_y, cur_angluar_z])  # 순수 gyro 값

                # navigation component of angulat rate with respect to ECEF(ECEF에 대한 nav frame의 angular rate)
                w_enn = np.array([v_e / (Rt + h), -v_n / (Rm + h), -v_e * math.tan(lat * degree) / (Rt + h)])
                rho_n, rho_e, rho_d = w_enn[0], w_enn[1], w_enn[2]

                # earth rotation angular vel(지구 자전 각속도)
                w_ien = np.array([Ome * math.cos(lat * degree), 0, - Ome * math.sin(lat * degree)])

                # using poistion and velocity, calculate gyro(angular vel)
                w_inn = np.array([w_ien[0] + w_enn[0], w_ien[1] + w_enn[1], w_ien[2] + w_enn[2]])
                Cbn = np.array(eul2DCM_bn(roll, pitch, yaw))  # body to ned DCM #검수완료

                # accel measurement #boty to nav #검수완료
                f_ned = Cbn @ np.array([[cur_accel_x], [cur_accel_y], [cur_accel_z]])  # body to ned accel

                f_n, f_e, f_d = f_ned[0][0], f_ned[1][0], f_ned[2][0]

                # mechanization # DCM differential equation #검수완료
                Cbn = Cbn + (Cbn @ np.array(skew(w_ibb[0] - dX[12][0], w_ibb[1] - dX[13][0], w_ibb[2] - dX[14][0])) -
                             np.array(skew(w_inn[0], w_inn[1], w_inn[2])) @ Cbn) * dt

                roll, pitch, yaw = DCM2eul_bn(Cbn)  # attitude update

                # -------------velocity update
                V_ned = np.array([v_n, v_e, v_d])

                m_7 = Cbn @ np.array([[cur_accel_x - dX[9][0]],
                                      [cur_accel_y - dX[10][0]],
                                      [cur_accel_z - dX[11][0]]]) - \
                      np.array(np.cross(np.array([w_ien[0] * 2, w_ien[1] * 2, w_ien[2] * 2]) +
                                        np.array([w_enn[0], w_enn[1], w_enn[2]]), V_ned)).reshape((-1, 1))

                V = V_ned.reshape((-1, 1)) + (m_7 + np.array([[0], [0], [g]])) * dt

                # velocity update
                # -------------velocity update

                # -------------Position update
                lat = lat + (180 / math.pi) * (0.5 * (V[0][0] + v_n) / (Rm + h)) * dt
                lon = lon + (180 / math.pi) * (0.5 * (V[1][0] + v_e) / ((Rt + h) * math.cos(lat * degree))) * dt
                h = h - (180 / math.pi) * (0.5 * (V[2][0] + v_d)) * dt

                # ---------------------------
                # ---------------------------
                # ---------------------------
                # INS Error Model
                # ---------------------------
                # ---------------------------
                # ---------------------------

                # -----------------constant_roll,constant_pitch
                X = np.array(
                    [lat, lon, h, 0.5 * (V[0][0] + v_n), 0.5 * (V[1][0] + v_e), 0.5 * (V[2][0] + v_d), roll, pitch,
                     yaw])  # next state

                # INS_Mechanization
                F = np.array(
                    INS_Mechanization(lat, h,
                                  Rm, Rt, Rmm, Rtt,
                                  rho_n, rho_e, rho_d,
                                  v_n, v_e, v_d,
                                  f_n, f_e, f_d,
                                  w_ien,
                                  Cbn))

                A = np.expm1(dt * F)
                # # -------------Prediction
                dX = A @ dX  # error state prediction
                P = A @ P @ np.transpose(A) + Q  # P prediction

                X_copy = X.tolist()
                X_copy = copy.deepcopy(X_copy)

                if GPS_Integration == 1:  # GPS 정보가 들어옴

                    GPS_dt = time.time() - GPS_start_time
                    GPS_start_time = time.time()  # i-1 시간 다시 갱신

                    # 255 인 경우는 적용안됨
                    N, E, D = pm.geodetic2ned(cur_lat, cur_lon, cur_alt, start_lat, start_lon, start_alt)

                    GPS_yaw = math.atan2(E - prev_E, N - prev_N) * radian

                    GPS_v_n, GPS_v_e, GPS_v_d = GPS_vn, GPS_ve, GPS_vd

                    B = H @ P @ np.transpose(H) + R
                    B = getMatrixInverse(B)
                    B = np.array(B)
                    K = P @ np.transpose(H) @ B  # Kalman gain

                    z = np.array([
                        [X_copy[0] - cur_lat],
                        [X_copy[1] - cur_lon],
                        [X_copy[2] - cur_alt],
                        [X_copy[3] - GPS_v_n],
                        [X_copy[4] - GPS_v_e],
                        [X_copy[5] - GPS_v_d]
                    ])

                    dX = dX + (K @ (z - (H @ dX)))

                    P = P - K @ H @ P

                    X[0] = X_copy[0] - dX[0][0]
                    X[1] = X_copy[1] - dX[1][0]
                    # X[2] = X_copy[2] - dX[2][0]
                    X[3] = X_copy[3] - dX[3][0]
                    X[4] = X_copy[4] - dX[4][0]
                    X[5] = X_copy[5] - dX[5][0]
                    X[2] = cur_alt
                    dX = copy.deepcopy(dX0)
                    prev_N, prev_E, prev_D = N, E, D

                dX = A @ dX  # error state prediction
                P = A @ P @ np.transpose(A) + Q  # P prediction

                if I_CNT == IMU_CNT.value and G_CNT_emergency == GPS_CNT.value:  # 통신 문제 발생

                    time.sleep(0.1)
                    if I_CNT == IMU_CNT.value and G_CNT_emergency == GPS_CNT.value:
                        print("Emergency!! Stop Integration")
                        break
                else:
                    I_CNT = IMU_CNT.value
                    G_CNT_emergency = GPS_CNT.value

                # 여기서 부터는 결과값을 애니메이션으로 출력시키는 부분!!!! 무시해도됨!!!!!
                #--------------------------------------------------------
                # --------------------------------------------------------
                # --------------------------------------------------------
                writer.writerow(X)
                vn100_writer.writerow(np.array([cur_lat, cur_lon, cur_alt,
                                                GPS_vn, GPS_ve, GPS_vd,
                                                cur_accel_x, cur_accel_y, cur_accel_z,
                                                cur_angluar_x, cur_angluar_y, cur_angluar_z,
                                                dt, GPS_dt, GPS_yaw, GPS_vn, GPS_ve, GPS_vd
                                                ]))

                error = GPS_yaw - X[8]

                kalman_ENU_E.value, kalman_ENU_N.value, kalman_ENU_U.value = pm.geodetic2enu(X[0], X[1], X[2],
                                                                                             start_lat, start_lon,
                                                                                             start_alt)

                Kalman_N, Kalman_E, Kalman_D = pm.geodetic2ned(X[0], X[1], X[2], start_lat, start_lon, start_alt)

                kalman_NED_N.value, kalman_NED_E.value, kalman_NED_D.value = Kalman_N, Kalman_E, Kalman_D

                GPS_NED_N_N, GPS_NED_E_E, GPS_NED_D_D = pm.geodetic2ned(float(cur_lat), float(cur_lon),
                                                                        float(cur_alt), start_lat, start_lon,
                                                                        start_alt)

                GPS_ENU_E_E, GPS_ENU_N_N, GPS_ENU_U_U = pm.geodetic2enu(float(cur_lat), float(cur_lon),
                                                                        float(cur_alt), start_lat, start_lon,
                                                                        start_alt)

                GPS_NED_N.value, GPS_NED_E.value, GPS_NED_D.value = GPS_NED_N_N, GPS_NED_E_E, GPS_NED_D_D

                GPS_ENU_E.value, GPS_ENU_N.value, GPS_ENU_U.value = GPS_ENU_E_E, GPS_ENU_N_N, GPS_ENU_U_U

                Kalman_comp_plot = np.append(Kalman_comp_plot, np.array([[Kalman_E, Kalman_N]]), axis=0)
                GPS_comp_plot = np.append(GPS_comp_plot, np.array([[GPS_NED_E_E, GPS_NED_N_N]]), axis=0)

                NED_data.writerow(np.array([Kalman_E, Kalman_N, GPS_NED_E_E, GPS_NED_N_N, error]))

                if show_animation:
                    plt.cla()
                    # for stopping simulation with the esc key.
                    plt.gcf().canvas.mpl_connect(
                        'key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])

                    try:
                        # ENU 확인

                        plt.plot(GPS_comp_plot[:, 0], GPS_comp_plot[:, 1], "or", markersize=3) # 현재 위치
                        plt.plot(Kalman_comp_plot[:, 0], Kalman_comp_plot[:, 1], "og", markersize=3)

                        plt.xlim(GPS_NED_E_E - 3, GPS_NED_E_E + 3)
                        plt.ylim(GPS_NED_N_N - 3, GPS_NED_N_N + 3)

                    except:
                        pass

                    plt.grid(True)
                    plt.pause(0.00001)

                # 여기서 부터는 결과값을 애니메이션으로 출력시키는 부분!!!! 무시해도됨!!!!!
                #--------------------------------------------------------
                # --------------------------------------------------------
                # --------------------------------------------------------



def INS_Mechanization(Lat, h, Rm, Rt, Rmm, Rtt, rho_n, rho_e, rho_d, v_n, v_e, v_d, f_n, f_e, f_d, w_ien, Cbn):
    degree = math.pi / 180

    cos_lat = math.cos(Lat * degree)
    tan_lat = math.tan(Lat * degree)
    sin_lat = math.sin(Lat * degree)

    F_pp = [
        [Rmm * rho_e / (Rm + h), 0, rho_e / (Rm + h)],
        [rho_n * (tan_lat - Rtt / (Rt + h)) / cos_lat, 0, -rho_n /(cos_lat * (Rt + h)) ],
        [0, 0, 0]
    ]

    F_pv = [
        [1 / (Rm + h), 0, 0],
        [0, 1 / (cos_lat * (Rt + h)), 0],
        [0, 0, -1]
    ]

    F_vp = [
        [Rmm * rho_e * v_d / (Rm + h) - (rho_n / ((cos_lat) ** 2) + 2 * w_ien[0]) * v_e - rho_n * rho_d * Rtt, 0,
         rho_e * v_d / (Rm + h) - rho_n * rho_d],
        [(2 * w_ien[0] + rho_n / ((cos_lat) ** 2) + rho_d * Rtt / (Rt + h)) * v_n - (
                    rho_n * Rtt / (Rt + h) - 2 * w_ien[2]) * v_d, 0, rho_d * v_n / (Rt + h) - rho_n * v_d / (Rt + h)],
        [(rho_n ** 2) * Rtt + (rho_e ** 2) * Rmm - 2 * w_ien[2] * v_e, 0, rho_n ** 2 + rho_e ** 2]

    ]

    F_vv = [[v_d / (Rm + h), 2 * rho_d + 2 * w_ien[2], -rho_e],
            [-2 * w_ien[2] - rho_d, (v_n * tan_lat + v_d) / (Rt + h), 2 * w_ien[0] + rho_n],
            [2 * rho_e, -2 * w_ien[0] - 2 * rho_n, 0]
            ]

    F_vphi = [
        [0, -f_d, f_e],
        [f_d, 0, -f_n],
        [-f_e, f_n, 0]
    ]

    F_phip = [
        [w_ien[2] - rho_n * Rtt / (Rt + h), 0, -rho_n / (Rt + h)],
        [-rho_e * Rmm / (Rm + h), 0, -rho_e / (Rm + h)],
        [-w_ien[0] - rho_n / (cos_lat) ** 2 - rho_d * Rtt / (Rt + h), 0, -rho_d / (Rt + h)]
    ]

    F_phiv = [
        [0, 1 / (Rt + h), 0],
        [-1 / (Rm + h), 0, 0],
        [0, -tan_lat / (Rt + h), 0]
    ]

    F_phiphi = [
        [0, w_ien[2] + rho_d, -rho_e],
        [-w_ien[2] - rho_d, 0, w_ien[0] + rho_n],
        [rho_e, -w_ien[0] - rho_n, 0]
    ]

    Cbn_minus = (-1) * Cbn

    F = [[0 for j in range(15)] for i in range(15)]

    for i in range(3):
        for k in range(3):
            F[i][k] = F_pp[i][k]

    for i in range(3):
        for k in range(3, 6):
            F[i][k] = F_pv[i][k - 3]

    for i in range(3, 6):
        for k in range(3):
            F[i][k] = F_vp[i - 3][k]

    for i in range(3, 6):
        for k in range(3, 6):
            F[i][k] = F_vv[i - 3][k - 3]

    for i in range(3, 6):
        for k in range(6, 9):
            F[i][k] = F_vphi[i - 3][k - 6]

    for i in range(3, 6):
        for k in range(9, 12):
            F[i][k] = Cbn[i - 3][k - 9]

    for i in range(6, 9):
        for k in range(0, 3):
            F[i][k] = F_phip[i - 6][k]

    for i in range(6, 9):
        for k in range(3, 6):
            F[i][k] = F_phiv[i - 6][k - 3]

    for i in range(6, 9):
        for k in range(6, 9):
            F[i][k] = F_phiphi[i - 6][k - 6]

    for i in range(6, 9):
        for k in range(12, 15):
            F[i][k] = Cbn_minus[i - 6][k - 12]


    return F


def transposeMatrix(m):
    return list(map(list,zip(*m)))

def getMatrixMinor(m, i, j):
    return [row[:j] + row[j + 1:] for row in (m[:i] + m[i + 1:])]

def getMatrixDeternminant(m):
    # base case for 2x2 matrix
    if len(m) == 2:
        return m[0][0] * m[1][1] - m[0][1] * m[1][0]

    determinant = 0
    for c in range(len(m)):
        determinant += ((-1) ** c) * m[0][c] * getMatrixDeternminant(getMatrixMinor(m, 0, c))
    return determinant

def getMatrixInverse( m):

    m = m.tolist()
    determinant = getMatrixDeternminant(m)
    # special case for 2x2 matrix:
    if len(m) == 2:
        return [[m[1][1] / determinant, -1 * m[0][1] / determinant],
                [-1 * m[1][0] / determinant, m[0][0] / determinant]]

    # find matrix of cofactors
    cofactors = []
    for r in range(len(m)):
        cofactorRow = []
        for c in range(len(m)):
            minor = getMatrixMinor(m, r, c)
            cofactorRow.append(((-1) ** (r + c)) * getMatrixDeternminant(minor))
        cofactors.append(cofactorRow)
    cofactors = transposeMatrix(cofactors)
    for r in range(len(cofactors)):
        for c in range(len(cofactors)):
            cofactors[r][c] = cofactors[r][c] / determinant
    return cofactors


def GPS_vel_Callback(data):
    GPS_fix_velocity_x.value = data.twist.twist.linear.x
    GPS_fix_velocity_y.value = data.twist.twist.linear.y
    GPS_fix_velocity_z.value = data.twist.twist.linear.z


def DCM2eul_bn(matrix):

    roll = math.atan2(matrix[2][1], matrix[2][2]) * 180 /math.pi #* math.pi / 180 이거 아니다.... 개고생함 ㅅㅂ
    pitch = math.asin(-matrix[2][0]) * 180 / math.pi  # * math.pi / 180
    yaw = math.atan2(matrix[1][0], matrix[0][0]) * 180 /math.pi

    return roll, pitch, yaw

def eul2DCM_bn(roll, pitch, yaw): # C ( b to n)
    degree = math.pi / 180
    matrix = [
        [math.cos(pitch * degree) * math.cos(yaw * degree),
         math.cos(yaw * degree) * math.sin(pitch * degree) * math.sin(roll * degree) -
         math.cos(roll * degree) * math.sin(yaw * degree),
         math.cos(roll * degree) * math.cos(yaw * degree) * math.sin(pitch * degree) +
         math.sin(roll * degree) * math.sin(yaw * degree)
         ],
        [math.cos(pitch * degree) * math.sin(yaw * degree),
         math.cos(roll * degree) * math.cos(yaw * degree) +
         math.sin(pitch * degree) * math.sin(roll * degree) * math.sin(yaw * degree),
         math.cos(roll * degree) * math.sin(pitch * degree) * math.sin(yaw * degree) -
         math.cos(yaw * degree) * math.sin(roll * degree)
         ],
        [-math.sin(pitch * degree),
         math.cos(pitch * degree) * math.sin(roll * degree),
         math.cos(pitch * degree) * math.cos(roll * degree)
         ]

    ]
    return matrix

def skew(x, y, z):
    M = [[0, -z, y],
         [z, 0, -x],
         [-y, x, 0]]

    return M

def GNSS_Subscribe():

    rospy.init_node('GNSS_Subscribe', anonymous=True)

    rospy.Subscriber("vectornav/IMU", Imu, chatterCallback)

    rospy.Subscriber("fix_RTK", NavSatFix, GPSCallback)

    rospy.Subscriber("/raw_data/fix_velocity", TwistWithCovarianceStamped, GPS_vel_Callback)

    rospy.spin()


if __name__ == '__main__':
    current_lat = Value('d', 0.0)
    current_lon = Value('d', 0.0)
    current_alt = Value('d', 0.0)
    current_accel_x = Value('d', 0.0)
    current_accel_y = Value('d', 0.0)
    current_accel_z = Value('d', 0.0)
    current_vel_x = Value('d', 0.0)
    current_vel_y = Value('d', 0.0)
    current_vel_z = Value('d', 0.0)
    current_quat_x = Value('d', 0.0)
    current_quat_y = Value('d', 0.0)
    current_quat_z = Value('d', 0.0)
    current_quat_w = Value('d', 0.0)
    current_yaw = Value('d', 0.0)
    

    #obj = [dist,x_cent, y_cent, x_min,x_max, y_min, y_max]
    obj1_dist = Value('d', 0.0)
    obj2_dist = Value('d', 0.0)
    obj3_dist = Value('d', 0.0)
    obj4_dist = Value('d', 0.0)
    
    obj1_x_cent = Value('d', 0.0)
    obj2_x_cent = Value('d', 0.0)
    obj3_x_cent = Value('d', 0.0)
    obj4_x_cent = Value('d', 0.0)

    obj1_y_cent = Value('d', 0.0)
    obj2_y_cent = Value('d', 0.0)
    obj3_y_cent = Value('d', 0.0)
    obj4_y_cent = Value('d', 0.0)

    obj1_x_min = Value('d', 0.0)
    obj2_x_min = Value('d', 0.0)
    obj3_x_min = Value('d', 0.0)
    obj4_x_min = Value('d', 0.0)

    obj1_x_max = Value('d', 0.0)
    obj2_x_max = Value('d', 0.0)
    obj3_x_max = Value('d', 0.0)
    obj4_x_max = Value('d', 0.0)

    obj1_y_min = Value('d', 0.0)
    obj2_y_min = Value('d', 0.0)
    obj3_y_min = Value('d', 0.0)
    obj4_y_min = Value('d', 0.0)

    obj1_y_max = Value('d', 0.0)
    obj2_y_max = Value('d', 0.0)
    obj3_y_max = Value('d', 0.0)
    obj4_y_max = Value('d', 0.0)

    IMU_CTC    = Value('d', 0.0)
    IMU_CNT    = Value('d', 0.0)
    GPS_CTC    = Value('d', 0.0)
    GPS_CNT    = Value('d', 0.0)
    LIDAR_CTC  = Value('d', 0.0)
    LIDAR_CNT  = Value('d', 0.0)
    
    LIDAR_obj_1 = Value('d', 0.0)
    LIDAR_obj_2 = Value('d', 0.0)
    LIDAR_obj_3 = Value('d', 0.0)

    LIDAR_obj_4 = Value('d', 0.0)

    current_roll = Value('d', 0.0)
    current_pitch = Value('d', 0.0)

    kalman_pitch = Value('d', 0.0)
    kalman_roll = Value('d', 0.0)
    kalman_yaw = Value('d', 0.0)
    kalman_lat = Value('d', 0.0)
    kalman_lon = Value('d', 0.0)
    kalman_alt = Value('d', 0.0)

    kalman_NED_N = Value('d', 0.0)
    kalman_NED_E = Value('d', 0.0)
    kalman_NED_D = Value('d', 0.0)

    kalman_ENU_E = Value('d', 0.0)
    kalman_ENU_N = Value('d', 0.0)
    kalman_ENU_U = Value('d', 0.0)



    GPS_NED_N = Value('d', 0.0)
    GPS_NED_E = Value('d', 0.0)
    GPS_NED_D = Value('d', 0.0)

    GPS_ENU_E = Value('d', 0.0)
    GPS_ENU_N = Value('d', 0.0)
    GPS_ENU_U = Value('d', 0.0)

    GPS_fix_velocity_x = Value('d', 0.0)
    GPS_fix_velocity_y = Value('d', 0.0)
    GPS_fix_velocity_z = Value('d', 0.0)

    processedQ = Queue()

    th3 = Process(target=vn100_Kalman_filter, args = (current_lat, current_lon, current_alt, current_accel_x,
    current_accel_y, current_accel_z,current_vel_x, current_vel_y,current_vel_z,current_yaw,current_roll,
                                                      current_pitch,GPS_CNT,kalman_yaw,GPS_CTC,
                                                      kalman_pitch, kalman_roll, kalman_lat,
                                                      kalman_lon, kalman_alt,kalman_NED_N, kalman_NED_E, kalman_NED_D,
                                                      kalman_ENU_E, kalman_ENU_N, kalman_ENU_U,
                                                      GPS_NED_N, GPS_NED_E, GPS_NED_D,
                                                      GPS_ENU_E, GPS_ENU_N, GPS_ENU_U,IMU_CNT,
                                                      GPS_fix_velocity_x,GPS_fix_velocity_y,GPS_fix_velocity_z))

    th3.start()

    th5 = Process(target=GNSS_Subscribe, args = ())
    th5.start()

    rospy.init_node('INS_Integration', anonymous=True)

    pub_p = rospy.Publisher('INS', Float32MultiArray, queue_size=100)
    pub_q = rospy.Publisher('ENU', Float32MultiArray, queue_size=30)
    pub_t = rospy.Publisher('NED', Float32MultiArray, queue_size=30)

    while True:

        INS_array = Float32MultiArray()
        ENU_array = Float32MultiArray()
        NED_array = Float32MultiArray()

        if rospy.is_shutdown():
            print('shutdown')
            break

        if kalman_lat.value != 0: # 칼만필터가 적용 되었을 때


            INS_array.data = [kalman_lat.value, kalman_lon.value, kalman_alt.value,
                              current_lat.value, current_lon.value, current_alt.value,
                              kalman_roll.value, kalman_pitch.value, kalman_yaw.value,
                              current_accel_x.value, current_accel_y.value, current_accel_z.value,
                              current_vel_x.value, current_vel_y.value, current_vel_z.value,
                              current_quat_x.value, current_quat_y.value, current_quat_z.value, current_quat_w.value,
                              GPS_NED_N.value, GPS_NED_E.value, GPS_NED_D.value,
                              GPS_ENU_E.value, GPS_ENU_N.value,GPS_ENU_U.value
                              ]

            ENU_array.data = [kalman_ENU_E.value, kalman_ENU_N.value, kalman_ENU_U.value]

            NED_array.data = [kalman_NED_N.value, kalman_NED_E.value, kalman_NED_D.value]



            pub_p.publish(INS_array)
            pub_q.publish(ENU_array)
            pub_t.publish(NED_array)

            time.sleep(0.01)

