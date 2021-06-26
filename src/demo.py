import numpy as np
from socket import socket, AF_INET, SOCK_DGRAM
from imu_ekf import *

def calc_u(gyro, dt):
    gyro = np.array([
        [gyro[0]],
        [gyro[1]],
        [gyro[2]]
    ])
    u = gyro * dt
    return u
    
def calc_z(acc):
    z = np.array([
        [np.arctan(acc[1]/acc[2])], 
        [-np.arctan(acc[0]/np.sqrt(acc[1]**2+acc[2]**2))]
        ])
    return z

def convert_euler_to_Rxyz(x):
    c1 = np.cos(x[0][0])
    s1 = np.sin(x[0][0])
    c2 = np.cos(x[1][0])
    s2 = np.sin(x[1][0])
    c3 = np.cos(x[2][0])
    s3 = np.sin(x[2][0])
    Rx = np.array([
        [1, 0, 0],
        [0, c1, -s1],
        [0, s1, c1],
    ])
    Ry = np.array([
        [c2, 0, s2],
        [0, 1, 0],
        [-s2, 0, c2],
    ])
    Rz = np.array([
        [c3, -s3, 0],
        [s3, c3, 0],
        [0, 0, 1],
    ])
    Rxyz = Rz @ Ry @ Rx
    return Rxyz

def main():
    # UDP settings
    HOST = ''   
    R_PORT = 4001
    S_PORT = 4002
    ADDRESS = "127.0.0.1"
    rs = socket(AF_INET, SOCK_DGRAM)
    ss = socket(AF_INET, SOCK_DGRAM)
    rs.bind((HOST, R_PORT))

    # ekf init
    dt = 0.01
    x = np.array([[0], [0], [0]])
    P = np.diag([1.74E-2*dt**2, 1.74E-2*dt**2, 1.74E-2*dt**2])

    acc = None
    ts_pre = None

    while True:
        r_msg, address = rs.recvfrom(8192)
        sensor_type = str(r_msg.decode('utf-8').split('\t')[1])
        ts = float(r_msg.decode('utf-8').split('\t')[0])
        data_x = float(r_msg.decode('utf-8').split('\t')[-1].split(',')[1])
        data_y = float(r_msg.decode('utf-8').split('\t')[-1].split(',')[2])
        data_z = float(r_msg.decode('utf-8').split('\t')[-1].split(',')[3])

        if sensor_type == 'ACC':
            # acc = np.array([data_x, data_y, data_z]) # Android
            acc = -np.array([data_x, data_y, data_z]) # iOS

        if sensor_type == 'GYRO' and acc is not None:
            gyro = np.array([data_x, data_y, data_z])
            
            if ts_pre is not None:
                dt = ts - ts_pre
                u = calc_u(gyro, dt)
                z = calc_z(acc)
                R = np.diag([1.0*dt**2, 1.0*dt**2])
                Q = np.diag([1.74E-2*dt**2, 1.74E-2*dt**2, 1.74E-2*dt**2])
                # ekf
                x, P = ekf(x, u, z, P, R, Q)
                # send to viz
                Rxyz = convert_euler_to_Rxyz(x)
                r11, r12, r13 = Rxyz[0][0], Rxyz[0][1], Rxyz[0][2]
                r21, r22, r23 = Rxyz[1][0], Rxyz[1][1], Rxyz[1][2]
                r31, r32, r33 = Rxyz[2][0], Rxyz[2][1], Rxyz[2][2]
                s_msg = str(r11)+','+str(r12)+','+str(r13)+','+str(r21)+','+str(r22)+','+str(r23)+','+str(r31)+','+str(r32)+','+str(r33)
                ss.sendto(s_msg.encode(), (ADDRESS, S_PORT))
            ts_pre = ts

    rs.close()
    ss.close()

if __name__ == '__main__':
    main()