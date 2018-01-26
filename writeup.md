# Model Predictive Control Project

## The Model:

Our model has four states parameters and two actuator parameters.
The status is composed of its position (x and y coordinates), orientation (psi) and velocity (v).

And the actuator inputs allows us to control the car state.
We consider two actuators, the steering angle (delta) and the acceleration (a) which is for throttle and break.

The update equation is represented by the followings.

```
# Update x coordinate
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt

# Update y cordinate
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt

# Update orientation
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt

# Update velocity
v_[t+1] = v[t] + a[t] * dt
```

## Timestep Length and Elapsed Duration (N & dt):

The prediction horizon is the duration over which future predictions are made.
We’ll refer to this as T.

T is the product of two other variables, N and dt.
N is the number of timesteps in the horizon.
dt is how much time elapses between actuations.

N, dt, and T are hyperparameters you will need to tune for each model predictive controller you build. However, there are some general guidelines. T should be as large as possible, while dt should be as small as possible.

According to the lecture, T should be a few seconds, at most.
But When I did some experiments with a condition that T is 1.25 seconds (N=25, dt=0.05), optimizing trajectory was failed, because the duration is too long to fit the polynomial equation.

To avoid the issue, I decided to make T shorter.
In the end, we choose N=15 and dt=0.05, that is, T is 0.75 seconds.

## Polynomial Fitting and MPC Preprocessing:
A 3-order polynomial would fit most trajectories.
We fitted a 3-order polynomial to the waypoints returned by the simulator with `polyfit` function to do that.
The polynomial was fitted after converting the waypoints from the map coordinate system to the car centered coordinate system.

## Model Predictive Control with Latency
In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system.
A realistic delay might be on the order of 100 milliseconds.
An actuation command would not be executed instantly.

The status of the vehicle would be change for the latency. 
So I change the status to solve with `FG_eval.operator()`.
For instance, the difference of x coordinate would be calculated by velocity multiplied by latency.
As well as, we can calculate other differences using the update equation as we discussed above.

```
	const double latency = 0.1;
	const double Lf = 2.67;
	double px_delayed = v * latency;
	double py_delayed = 0;
	double psi_delayed = v / Lf * (- steer_value) * latency;
	double v_delayed = v + throttle_value * latency;
	double cte_delayed = cte + v * sin(epsi) * latency;
	double epsi_delayed = epsi + psi_delayed;
	Eigen::VectorXd state_delayed(6);
	state_delayed << px_delayed, py_delayed, psi_delayed, v_delayed, cte_delayed, epsi_delayed;

	// Find the optimal actuator values.
	auto vars = mpc.Solve(state_delayed, coeffs);
	steer_value = vars[0]/deg2rad(25)/Lf;
	throttle_value = vars[1];
```
