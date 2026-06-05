import numpy as np
import matplotlib.pyplot as plt

np.random.seed(1)

dt = 0.1
N = 300
p_ref = 10.0

A = np.array([
    [1, dt],
    [0, 1]
])

B = np.array([
    [0.5 * dt**2],
    [dt]
])

C = np.array([
    [1, 0]
])

Q = np.array([
    [0.001, 0],
    [0, 0.001]
])

R = np.array([[0.5**2]])  # sensor std = 0.5 m

kp = 1.5
kd = 2.0

def kalman_filter_step(x_hat, P, u_prev, y, A, B, C, Q, R):
    # Predict
    x_minus = A @ x_hat + B * u_prev
    P_minus = A @ P @ A.T + Q

    # Correct
    innovation = y - C @ x_minus
    S = C @ P_minus @ C.T + R
    K = P_minus @ C.T @ np.linalg.inv(S)

    x_hat = x_minus + K @ innovation
    P = (np.eye(2) - K @ C) @ P_minus

    return x_hat, P, K

# True systems
x_raw = np.array([[0.0], [0.0]])
x_kf = np.array([[0.0], [0.0]])

# Kalman initial estimate
x_hat = np.array([[0.0], [0.0]])
P = np.eye(2) * 10

u_raw_prev = 0.0
u_kf_prev = 0.0
y_raw_prev = 0.0

time = []
p_raw_history = []
p_kf_history = []
y_raw_history = []
p_hat_history = []
v_raw_history = []
v_hat_history = []
u_raw_history = []
u_kf_history = []
P_position_history = []
P_velocity_history = []

for k in range(N):
    t = k * dt

    # Same sensor noise level for both systems
    noise_raw = np.random.normal(0, np.sqrt(R[0, 0]))
    noise_kf = np.random.normal(0, np.sqrt(R[0, 0]))

    # Measurements
    y_raw = C @ x_raw + noise_raw
    y_kf = C @ x_kf + noise_kf

    # Raw velocity from finite difference
    if k == 0:
        v_raw = 0.0
    else:
        v_raw = (y_raw[0, 0] - y_raw_prev) / dt

    # Controller using raw noisy measurement
    u_raw = kp * (p_ref - y_raw[0, 0]) - kd * v_raw

    # Kalman filter estimate
    x_hat, P, K = kalman_filter_step(
        x_hat, P, u_kf_prev, y_kf, A, B, C, Q, R
    )

    p_hat = x_hat[0, 0]
    v_hat = x_hat[1, 0]

    # Controller using Kalman estimate
    u_kf = kp * (p_ref - p_hat) - kd * v_hat

    # Limit acceleration command
    u_raw = np.clip(u_raw, -5, 5)
    u_kf = np.clip(u_kf, -5, 5)

    # Apply control to the real systems
    x_raw = A @ x_raw + B * u_raw
    x_kf = A @ x_kf + B * u_kf

    # Save previous values
    y_raw_prev = y_raw[0, 0]
    u_raw_prev = u_raw
    u_kf_prev = u_kf

    # Store data
    time.append(t)
    p_raw_history.append(x_raw[0, 0])
    p_kf_history.append(x_kf[0, 0])
    y_raw_history.append(y_raw[0, 0])
    p_hat_history.append(p_hat)
    v_raw_history.append(v_raw)
    v_hat_history.append(v_hat)
    u_raw_history.append(u_raw)
    u_kf_history.append(u_kf)
    P_position_history.append(P[0, 0])
    P_velocity_history.append(P[1, 1])

plt.figure()
plt.plot(time, p_raw_history, label="Position: raw controller")
plt.plot(time, p_kf_history, label="Position: KF controller")
plt.axhline(p_ref, linestyle="--", label="Reference")
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.legend()
plt.grid()
plt.show()

plt.figure()
plt.plot(time, u_raw_history, label="Control input: raw measurement")
plt.plot(time, u_kf_history, label="Control input: Kalman estimate")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration command")
plt.legend()
plt.grid()
plt.show()

plt.figure()
plt.plot(time, v_raw_history, label="Raw velocity estimate")
plt.plot(time, v_hat_history, label="Kalman velocity estimate")
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m/s]")
plt.legend()
plt.grid()
plt.show()

plt.figure()
plt.plot(time, P_position_history, label="Position uncertainty P[0,0]")
plt.plot(time, P_velocity_history, label="Velocity uncertainty P[1,1]")
plt.xlabel("Time [s]")
plt.ylabel("Uncertainty")
plt.legend()
plt.grid()
plt.show()


print("Std of raw control:", np.std(u_raw_history))
print("Std of KF control:", np.std(u_kf_history))
print("Final raw position:", p_raw_history[-1])
print("Final KF position:", p_kf_history[-1])

