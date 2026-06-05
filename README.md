# Kalman Filter for 1D Robot Control

This repository demonstrates the basic theory and Python implementation of a **Kalman filter** and shows how it can improve a simple feedback controller.

The example uses a 1D robot moving along a straight line. The robot has two states:

$$
x =
\begin{bmatrix}
p \\
v
\end{bmatrix}
$$

where:

- $p$ is position
- $v$ is velocity

The sensor measures only position, and the position measurement is noisy. The Kalman filter estimates both position and velocity, then the controller uses the estimated state instead of raw noisy measurements.

---

## 1. Why We Need a Kalman Filter in Control

In control systems, the controller usually needs the system state in order to compute a control command.

For example, a simple feedback controller may use:

$$
u = f(x)
$$

or, for a linear state-feedback controller:

$$
u = -Kx
$$

However, in real systems, the true state $x$ is usually not known perfectly.

Sensors are noisy, and sometimes they do not measure all states directly. For example, a robot may have a position sensor, but not a direct velocity sensor.

So instead of knowing the true state:

$$
x_k
$$

we only measure:

$$
y_k = Cx_k + n_k
$$

where:

- $y_k$ is the sensor measurement
- $C$ is the measurement matrix
- $n_k$ is measurement noise

The Kalman filter solves this problem by estimating the true state:

$$
\hat{x}_k \approx x_k
$$

The controller then uses:

$$
u = f(\hat{x})
$$

instead of using noisy raw measurements directly.

---

## 2. Core Idea of the Kalman Filter

A Kalman filter combines two sources of information:

1. **Prediction from the model**
2. **Correction from the sensor measurement**

The model predicts what the state should be. The sensor tells us what was measured. The Kalman filter combines both using a weighting factor called the **Kalman gain**.

The main update equation is:

$$
\hat{x}_k = \hat{x}_k^- + K_k(y_k - C\hat{x}_k^-)
$$

where:

- $\hat{x}_k^-$ is the predicted state estimate
- $\hat{x}_k$ is the corrected state estimate
- $K_k$ is the Kalman gain
- $y_k$ is the measurement
- $C\hat{x}_k^-$ is the predicted measurement
- $y_k - C\hat{x}_k^-$ is the innovation, or measurement residual

In words:

$$
\text{New estimate} = \text{Prediction} + \text{Kalman gain} \times \text{Measurement error}
$$

The Kalman gain decides how much the filter should trust the sensor compared to the model.

---

## 3. 1D Robot State-Space Model

For a robot moving in one dimension, define the state as:

$$
x_k =
\begin{bmatrix}
p_k \\
v_k
\end{bmatrix}
$$

where:

- $p_k$ is position at time step $k$
- $v_k$ is velocity at time step $k$

Assume the control input is acceleration:

$$
u_k = a_k
$$

Using basic kinematics:

$$
p_{k+1} = p_k + v_k\Delta t + \frac{1}{2}u_k\Delta t^2
$$

$$
v_{k+1} = v_k + u_k\Delta t
$$

These equations can be written in state-space form:

$$
x_{k+1} = Ax_k + Bu_k
$$

where:

$$
A =
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}
$$

and:

$$
B =
\begin{bmatrix}
\frac{1}{2}\Delta t^2 \\
\Delta t
\end{bmatrix}
$$

The matrix $A$ comes directly from the motion equations.

For the position equation:

$$
p_{k+1} = 1p_k + \Delta t v_k
$$

so the first row of $A$ is:

$$
\begin{bmatrix}
1 & \Delta t
\end{bmatrix}
$$

For the velocity equation:

$$
v_{k+1} = 0p_k + 1v_k
$$

so the second row of $A$ is:

$$
\begin{bmatrix}
0 & 1
\end{bmatrix}
$$

Therefore:

$$
A =
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}
$$

If $\Delta t = 0.1$, then:

$$
A =
\begin{bmatrix}
1 & 0.1 \\
0 & 1
\end{bmatrix}
$$

and:

$$
B =
\begin{bmatrix}
\frac{1}{2}(0.1)^2 \\
0.1
\end{bmatrix}
=
\begin{bmatrix}
0.005 \\
0.1
\end{bmatrix}
$$

---

## 4. Measurement Model

In this example, the sensor measures only position. It does not directly measure velocity.

The measurement equation is:

$$
y_k = Cx_k + n_k
$$

where $n_k$ is measurement noise.

Because only position is measured:

$$
C =
\begin{bmatrix}
1 & 0
\end{bmatrix}
$$

This gives:

$$
y_k =
\begin{bmatrix}
1 & 0
\end{bmatrix}
\begin{bmatrix}
p_k \\
v_k
\end{bmatrix}
= p_k
$$

So $C$ selects the position from the state vector.

---

## 5. Process Noise and Measurement Noise

The real system is not perfect, so we include noise.

The full system model is:

$$
x_{k+1} = Ax_k + Bu_k + w_k
$$

The measurement model is:

$$
y_k = Cx_k + n_k
$$

where:

- $w_k$ is process noise
- $n_k$ is measurement noise

The Kalman filter uses two important covariance matrices:

$$
Q = \text{process noise covariance}
$$

$$
R = \text{measurement noise covariance}
$$

### Meaning of $Q$

$Q$ represents uncertainty in the model.

For example, $Q$ should be larger if:

- the robot slips
- there is wind or disturbance
- the actuator does not produce the exact commanded acceleration
- the model is too simple
- friction or unknown dynamics are ignored

Large $Q$ means:

$$
\text{Trust the model less}
$$

Small $Q$ means:

$$
\text{Trust the model more}
$$

### Meaning of $R$

$R$ represents uncertainty in the sensor measurement.

For example, $R$ should be larger if:

- the position sensor is noisy
- GPS is inaccurate
- measurements jump around a lot

Large $R$ means:

$$
\text{Trust the sensor less}
$$

Small $R$ means:

$$
\text{Trust the sensor more}
$$

A useful memory trick:

$$
R = \text{How bad is my sensor?}
$$

$$
Q = \text{How wrong is my model?}
$$

---

## 6. Estimation Uncertainty Matrix $P$

The Kalman filter does not only estimate the state. It also keeps track of how uncertain the estimate is.

The state estimate is:

$$
\hat{x}_k =
\begin{bmatrix}
\hat{p}_k \\
\hat{v}_k
\end{bmatrix}
$$

The uncertainty matrix is:

$$
P_k =
\begin{bmatrix}
P_{pp} & P_{pv} \\
P_{vp} & P_{vv}
\end{bmatrix}
$$

where:

- $P_{pp}$ is uncertainty in the position estimate
- $P_{vv}$ is uncertainty in the velocity estimate
- $P_{pv}$ and $P_{vp}$ describe correlation between position and velocity uncertainty

In Python:

```python
x_hat[0, 0]  # estimated position
x_hat[1, 0]  # estimated velocity

P[0, 0]      # uncertainty of estimated position
P[1, 1]      # uncertainty of estimated velocity
```

Important distinction:

$$
\hat{x} = \text{what I think the state is}
$$

$$
P = \text{how uncertain I am about that estimate}
$$

So if $P[1,1]$ becomes smaller, it does **not** mean velocity is decreasing. It means the filter is becoming more confident about the velocity estimate.

---

## 7. Kalman Filter Equations

The Kalman filter repeats two steps:

1. Prediction
2. Correction

---

### Step 1: Prediction

Predict the next state:

$$
\hat{x}_k^- = A\hat{x}_{k-1} + Bu_{k-1}
$$

Predict the uncertainty:

$$
P_k^- = AP_{k-1}A^T + Q
$$

The $+Q$ term is added because the model is not perfect. Prediction moves the estimate forward, and $Q$ admits that the prediction may be wrong.

---

### Step 2: Correction

Compute the innovation:

$$
y_k - C\hat{x}_k^-
$$

This is:

$$
\text{actual measurement} - \text{predicted measurement}
$$

Compute the innovation covariance:

$$
S_k = CP_k^-C^T + R
$$

This includes both:

- uncertainty from the predicted state
- uncertainty from the sensor noise

So:

$$
S_k = \text{prediction uncertainty in measurement space} + \text{sensor noise}
$$

Compute the Kalman gain:

$$
K_k = P_k^-C^T(CP_k^-C^T + R)^{-1}
$$

or using $S_k$:

$$
K_k = P_k^-C^T S_k^{-1}
$$

Update the state estimate:

$$
\hat{x}_k = \hat{x}_k^- + K_k(y_k - C\hat{x}_k^-)
$$

Update the uncertainty:

$$
P_k = (I - K_kC)P_k^-
$$

A more numerically stable form is the Joseph form:

$$
P_k = (I - K_kC)P_k^-(I - K_kC)^T + K_kRK_k^T
$$

The simplified form is used in this project for clarity.

---

## 8. Where the Kalman Gain Comes From

The Kalman gain comes from minimizing the expected estimation error.

The estimation error is:

$$
e_k = x_k - \hat{x}_k
$$

The error covariance is:

$$
P_k = E[e_ke_k^T]
$$

The Kalman filter chooses $K_k$ to make this uncertainty as small as possible.

The gain equation is:

$$
K_k = P_k^-C^T(CP_k^-C^T + R)^{-1}
$$

Interpretation:

$$
K_k =
\frac{\text{state-measurement uncertainty relation}}
{\text{total measurement uncertainty}}
$$

For a simple scalar case, the gain becomes:

$$
K = \frac{P^-}{P^- + R}
$$

If sensor noise $R$ is large:

$$
K = \frac{P^-}{P^- + \text{large}}
$$

then $K$ becomes small, so the filter trusts the model more.

If model uncertainty $P^-$ is large:

$$
K = \frac{\text{large}}{\text{large} + R}
$$

then $K$ becomes large, so the filter trusts the measurement more.

Summary:

$$
R \uparrow \Rightarrow K \downarrow \Rightarrow \text{trust sensor less}
$$

$$
Q \uparrow \Rightarrow P^- \uparrow \Rightarrow K \uparrow \Rightarrow \text{trust model less}
$$

---

## 9. Why Raw Velocity from Noisy Position Is Bad

If the sensor only measures position, a simple way to estimate velocity is finite difference:

$$
v_{raw} = \frac{y_k - y_{k-1}}{\Delta t}
$$

However, if the position measurement is noisy, this velocity estimate becomes very noisy.

For example:

$$
p_{k-1}=10.0, \quad p_k=10.1
$$

with:

$$
\Delta t=0.1
$$

then:

$$
v = \frac{10.1-10.0}{0.1}=1.0
$$

But if noise changes the measurements to:

$$
y_{k-1}=9.8, \quad y_k=10.3
$$

then:

$$
v_{raw} = \frac{10.3-9.8}{0.1}=5.0
$$

The robot did not really become five times faster. The noise created a fake velocity.

This is why the Kalman filter is useful: it estimates velocity using both the motion model and the noisy position measurement.

---

## 10. Controller Using the Kalman Filter

The goal is to move the robot to a reference position:

$$
p_{ref}=10
$$

A simple PD controller is:

$$
u = k_p(p_{ref} - p) - k_dv
$$

where:

- $k_p(p_{ref}-p)$ pulls the robot toward the target
- $-k_dv$ acts like damping or braking

In a real system, the true position and velocity are not known, so the controller should use the Kalman estimate:

$$
u_{KF} = k_p(p_{ref} - \hat{p}) - k_d\hat{v}
$$

The raw measurement controller uses:

$$
u_{raw} = k_p(p_{ref} - y) - k_dv_{raw}
$$

where:

$$
v_{raw} = \frac{y_k - y_{k-1}}{\Delta t}
$$

Because $v_{raw}$ is noisy, $u_{raw}$ becomes jumpy.

The Kalman controller uses:

$$
\hat{x} =
\begin{bmatrix}
\hat{p} \\
\hat{v}
\end{bmatrix}
$$

so the control input is smoother:

$$
u_{KF} = k_p(p_{ref} - \hat{p}) - k_d\hat{v}
$$

---

## 11. Python Implementation

The main matrices are defined as:

```python
dt = 0.1

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

R = np.array([[0.5**2]])
```

Here:

```python
R = np.array([[0.5**2]])
```

means the sensor standard deviation is $0.5$ m, so the variance is:

$$
R = \sigma^2 = 0.5^2 = 0.25
$$

This is why the simulation generates noise using:

```python
noise = np.random.normal(0, np.sqrt(R[0, 0]))
```

because `np.random.normal()` expects standard deviation, not variance.

---

## 12. Kalman Filter Step Function

```python
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
```

Explanation line by line:

```python
x_minus = A @ x_hat + B * u_prev
```

This predicts the next state:

$$
\hat{x}_k^- = A\hat{x}_{k-1}+Bu_{k-1}
$$

```python
P_minus = A @ P @ A.T + Q
```

This predicts the next uncertainty:

$$
P_k^- = AP_{k-1}A^T + Q
$$

```python
innovation = y - C @ x_minus
```

This computes the measurement prediction error:

$$
y_k - C\hat{x}_k^-
$$

```python
S = C @ P_minus @ C.T + R
```

This computes innovation uncertainty:

$$
S_k = CP_k^-C^T + R
$$

```python
K = P_minus @ C.T @ np.linalg.inv(S)
```

This computes the Kalman gain:

$$
K_k = P_k^-C^TS_k^{-1}
$$

```python
x_hat = x_minus + K @ innovation
```

This corrects the state estimate:

$$
\hat{x}_k = \hat{x}_k^- + K_k(y_k-C\hat{x}_k^-)
$$

```python
P = (np.eye(2) - K @ C) @ P_minus
```

This corrects the uncertainty:

$$
P_k = (I-K_kC)P_k^-
$$

---

## 13. Expected Results

The simulation compares two controllers:

1. Controller using raw noisy position and finite-difference velocity
2. Controller using Kalman-filtered position and velocity

The expected behavior is:

$$
v_{raw} \text{ is noisy}
$$

$$
\hat{v} \text{ is smoother}
$$

and therefore:

$$
u_{raw} \text{ is jumpy}
$$

$$
u_{KF} \text{ is smoother}
$$

The Kalman filter improves the controller because the controller does not react directly to sensor noise.

A useful numerical comparison is:

```python
print("Std of raw control:", np.std(u_raw_history))
print("Std of KF control:", np.std(u_kf_history))
print("Final raw position:", p_raw_history[-1])
print("Final KF position:", p_kf_history[-1])
```

Usually:

$$
\text{std}(u_{raw}) > \text{std}(u_{KF})
$$

because the raw controller reacts more strongly to noisy measurements.

---

## 14. Tuning $Q$ and $R$

### Increasing $R$

If you increase $R$:

```python
R = np.array([[2.0**2]])
```

then the filter assumes the sensor is very noisy.

Result:

$$
R \uparrow \Rightarrow K \downarrow \Rightarrow \text{trust sensor less}
$$

The estimate becomes smoother but may respond more slowly.

### Decreasing $R$

If you decrease $R$:

```python
R = np.array([[0.1**2]])
```

then the filter assumes the sensor is accurate.

Result:

$$
R \downarrow \Rightarrow K \uparrow \Rightarrow \text{trust sensor more}
$$

The estimate follows the measurement more closely, but it may become noisier.

### Increasing $Q$

If you increase $Q$, the filter assumes the model is less reliable.

Result:

$$
Q \uparrow \Rightarrow K \uparrow \Rightarrow \text{trust model less and sensor more}
$$

This is useful if the robot slips or the model is inaccurate.

### Decreasing $Q$

If you decrease $Q$, the filter assumes the model is reliable.

Result:

$$
Q \downarrow \Rightarrow \text{trust model more}
$$

The estimate becomes smoother but may be too slow to react to real changes.

---

## 15. How to Run

Install dependencies:

```bash
pip install numpy matplotlib
```

Run the simulation:

```bash
python3 1Drobot.py
```

The program will show plots comparing:

- raw controller position vs Kalman-filtered controller position
- raw control input vs Kalman-filtered control input
- raw velocity estimate vs Kalman velocity estimate
- position and velocity uncertainty from $P$

---

## 16. Repository Structure

```text
kalman_filter/
├── 1Drobot.py
├── README.md
└── .gitignore
```

---

## 17. Main Takeaways

The Kalman filter is useful in control because it estimates the true state of a system from noisy measurements.

It combines:

$$
\text{model prediction}
$$

with:

$$
\text{sensor correction}
$$

using the Kalman gain:

$$
K_k = P_k^-C^T(CP_k^-C^T+R)^{-1}
$$

The most important intuition is:

$$
\hat{x}_k = \hat{x}_k^- + K_k(y_k-C\hat{x}_k^-)
$$

or:

$$
\text{Estimate} = \text{Prediction} + \text{Trust factor} \times \text{Measurement error}
$$

For control, the Kalman filter allows the controller to use:

$$
\hat{p}, \hat{v}
$$

instead of noisy raw measurements.

This usually gives:

- smoother velocity estimates
- smoother control commands
- less actuator chattering
- better behavior when sensors are noisy
- a cleaner separation between estimation and control

---

## 18. Control Loop Summary

The complete control structure is:

```text
True System / Plant
        ↓
Noisy Sensor Measurement
        ↓
Kalman Filter
        ↓
Estimated State
        ↓
Controller
        ↓
Control Input
        ↓
True System / Plant
```

Mathematically:

$$
x_{k+1} = Ax_k + Bu_k + w_k
$$

$$
y_k = Cx_k + n_k
$$

$$
\hat{x}_k = \text{KalmanFilter}(y_k,u_{k-1})
$$

$$
u_k = k_p(p_{ref}-\hat{p}_k)-k_d\hat{v}_k
$$

This is a basic example of using a state estimator inside a feedback control loop.
