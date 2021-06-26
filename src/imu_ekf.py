import numpy as np

def f(x, u):
    u_x, u_y, u_z = u[0][0], u[1][0], u[2][0]
    c1, s1 = np.cos(x[0][0]), np.sin(x[0][0])
    c2, s2 = np.cos(x[1][0]), np.sin(x[1][0])
    c3, s3 = np.cos(x[2][0]), np.sin(x[2][0])
    x = np.array([
        [x[0][0]+u_x+u_y*s1*s2/c2+u_z*c1*s2/c2],
        [x[1][0]+u_y*c1-u_z*s1],
        [x[2][0]+u_y*s1/c2+u_z*c1/c2]
    ])
    return x

def h(x):
    y = np.eye(2, 3) @ x
    return y

def predict_x(x, u):
    x = f(x, u)
    return x

def predict_P(P, F, Q):
    P = F @ P @ F.T + Q
    return P

def calc_F(x, u):
    u_x, u_y, u_z = u[0][0], u[1][0], u[2][0]
    c1, s1 = np.cos(x[0][0]), np.sin(x[0][0])
    c2, s2 = np.cos(x[1][0]), np.sin(x[1][0])
    c3, s3 = np.cos(x[2][0]), np.sin(x[2][0])
    F = np.array([
        [1+u_y*c1*s2/c2-u_z*s1*s2/c2, u_y*s1/c2**2+u_z*c1/c2**2, 0],
        [-u_y*s1-u_z*c1, 1, 0],
        [u_y*c1/c2-u_z*s1/c2, u_y*s1*s2/c2**2+u_z*c1*s2/c2**2, 1]
    ])
    return F

def calc_H():
    H = np.eye(2, 3)
    return H

def update_y_res(z, x):
    y_res = z - h(x)
    return y_res

def update_S(P, H, R):
    S = H @ P @ H.T + R
    return S

def update_K(P, H, S):
    K = P @ H.T @ np.linalg.inv(S)
    return K

def update_x(x, y_res, K):
    x = x + K @ y_res
    return x

def update_P(P, H, K):
    I = np.identity(3)
    P = (I - K @ H) @ P
    return P

def ekf(x, u, z, P, R, Q):
    # Predict
    F = calc_F(x, u)
    x = predict_x(x, u)
    H = calc_H()
    P = predict_P(P, F, Q)

    # Update
    y_res = update_y_res(z, x)
    S = update_S(P, H, R)
    K = update_K(P, H, S)
    x = update_x(x, y_res, K)
    P = update_P(P, H, K)
    return x, P

