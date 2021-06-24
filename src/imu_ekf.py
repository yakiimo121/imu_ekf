import numpy as np
from socket import socket, AF_INET, SOCK_DGRAM

class ExtendedKalmanFilter():

    def __init__(self, x, P):
        self.x = x
        self.P = P

    def f(self, x, u):
        R = np.array([
            [1, np.sin(x[0][0])*np.tan(x[1][0]), np.cos(x[0][0])*np.tan(x[1][0])],
            [0, np.cos(x[0][0]), -np.sin(x[0][0])],
            [0, np.sin(x[0][0])/np.cos(x[1][0]), np.cos(x[0][0])/np.cos(x[1][0])]
            ])
        x = x + R @ u
        return x

    def h(self, x):
        y = np.eye(2, 3) @ x
        return y

    def predict_x(self, x, u):
        x = self.f(x, u)
        return x

    def predict_P(self, P, F, Q):
        P = F @ P @ F.T + Q
        return P

    def calulate_F(self, x, u):
        F = np.array([
            [1+u[1][0]*np.cos(x[0][0])*np.tan(x[1][0])-u[2][0]*np.sin(x[0][0])*np.tan(x[1][0]), u[1][0]*np.sin(x[0][0])/np.cos(x[1][0])**2+u[2][0]*np.cos(x[0][0])/np.cos(x[1][0])**2, 0],
            [-u[1][0]*np.sin(x[0][0])-u[2][0]*np.cos(x[0][0]), 1, 0],
            [-u[1][0]*np.cos(x[0][0])/np.cos(x[1][0])-u[2][0]*np.sin(x[0][0])/np.cos(x[1][0]), -u[1][0]*np.sin(x[0][0])*np.sin(x[1][0])/np.cos(x[1][0])**2+u[2][0]*np.sin(x[0][0])*np.sin(x[1][0])/np.cos(x[1][0])**2, 1],
        ])
        return F

    def calulate_H(self):
        H = np.eye(2, 3)
        return H

    def update_y_res(self, z, x):
        y_res = z - self.h(x)
        return y_res

    def update_S(self, P, H, R):
        S = H @ P @ H.T + R
        return S

    def update_K(self, P, H, S):
        K = P @ H.T @ np.linalg.inv(S.astype(np.float64))
        return K

    def update_x(self, x, y_res, K):
        x = x + K @ y_res
        return x

    def update_P(self, P, H, K):
        I = np.identity(3)
        P = (I - K @ H) @ P
        return P

    def ekf(self, x, u, z, P, R, Q):
        # Predict
        # print(x)
        F = self.calulate_F(x, u)
        # print('F', F)
        x = self.predict_x(x, u)
        # print('x', x)
        H = self.calulate_H()
        # print('H', H)
        P = self.predict_P(P, F, Q)
        # print('P', P)

        # Update
        y_res = self.update_y_res(z, x)
        # print('y_res', y_res)
        S = self.update_S(P, H, R)
        # print('S', S)
        K = self.update_K(P, H, S)
        # print('K', K)
        x = self.update_x(x, y_res, K)
        # print('x', x)
        P = self.update_P(P, H, K)
        # print('P', P)
        return x, P

def calculate_u(gyro, dt):
    gyro = np.array([
        [gyro[0]],
        [gyro[1]],
        [gyro[2]]
    ])
    u = gyro * dt
    return u
    
def calculate_z(acc):
    z = np.array([
        [np.arctan(acc[1]/acc[2])], 
        [-np.arctan(acc[0]/np.sqrt(acc[1]**2+acc[2]**2))]
        ])
    return z

def convert_x_to_rmatrix(x):
    c1 = np.cos(x[0][0])
    s1 = np.sin(x[0][0])
    c2 = np.cos(x[1][0])
    s2 = np.sin(x[1][0])
    c3 = np.cos(x[2][0])
    s3 = np.sin(x[2][0])
    r11 = c2*c3
    r12 = s1*s3+c1*c3*s2
    r13 = c3*s1*s2-c1*s3
    r21 = -s2
    r22 = c1*c2
    r23 = c2*s1
    r31 = c2*s3
    r32 = c1*s2*s3-c3*s1
    r33 = c1*c3+s1*s2*s3
    return r11, r12, r13, r21, r22, r23, r31, r32, r33

def main():
    HOST = ''   
    R_PORT = 4001
    S_PORT = 4002
    ADDRESS = "127.0.0.1"

    rs = socket(AF_INET, SOCK_DGRAM)
    ss = socket(AF_INET, SOCK_DGRAM)
    
    rs.bind((HOST, R_PORT))


    dt = 0.01
    x = np.array([[0], [0], [0]])
    P = np.diag([1.0*dt**2, 1.0*dt**2, 1.0*dt**2])

    ekf = ExtendedKalmanFilter(x, P)

    acc = None
    gyro = None

    while True:
        print(x)
        r_msg, address = rs.recvfrom(8192)
        print(r_msg)
        sensor_type = str(r_msg.decode('utf-8').split('\t')[1])
        # ts = int(float(r_msg.decode('utf-8').split('\t')[0]))
        data_x = float(r_msg.decode('utf-8').split('\t')[-1].split(',')[1])
        data_y = float(r_msg.decode('utf-8').split('\t')[-1].split(',')[2])
        data_z = float(r_msg.decode('utf-8').split('\t')[-1].split(',')[3])

        if sensor_type == 'ACC':
            acc = np.array([data_x, data_y, data_z])

        if sensor_type == 'GYRO':
            gyro = np.array([data_x, data_y, data_z])

        if acc is not None and gyro is not None:
            u = calculate_u(gyro, dt)
            z = calculate_z(acc)
            R = np.diag([1.74E-2*dt**2, 1.0*dt**2])
            Q = np.diag([1.0*dt**2, 1.0*dt**2, 1.0*dt**2])
            x, P = ekf.ekf(x, u, z, P, R, Q)

            r11, r12, r13, r21, r22, r23, r31, r32, r33 = convert_x_to_rmatrix(x)
            s_msg = str(r11)+','+str(r12)+','+str(r13)+','+str(r21)+','+str(r22)+','+str(r23)+','+str(r31)+','+str(r32)+','+str(r33)
            ss.sendto(s_msg.encode(), (ADDRESS, S_PORT))

    rs.close()
    ss.close()


if __name__ == '__main__':
    main()