# **Extended Kalman Filters**
## Project Writeup

### 1. Writeup

This writeup describes implementation steps that were taken to address project rubric points.

### 2. Implementation

I have started implementation from CalculateRMSE and CalculateJacobian functions in *tools.cpp*.
Then in *FusionEKF.cpp* I have created laser measurement matrix *H<sub>laser</sub>* and initialized KalmanFilter with initial values.
Most matrices are created inside KalmanFilter Init function as they have a common element scheme, like identity matrix,
or yet to be changed on the next measurement processing. The ones initialized in FusionEKF are - state vector *x* and
state covariance matrix *P*. For the radar measurements a conversion from polar to cartesian coordinates is made using the following equations:

*x = ρ⋅cos(φ)*

*y = ρ⋅sin(φ)*

On the next measurement, *Δt* is calculated, and the state transition and process noise covariance matrices are updated.
After this, I run the Kalman filter prediction step and then, depending on the type of the sensor, measurement step: standard Kalman filter for laser
and extended Kalman filter for radar, for which a Jacobian matrix is calculated. At this step I am passing corresponding measurement matrix *H* and
measurement covariance matrix *R*, this was enforced to keep consistency between these parameters and type of Kalman filter used.

In the *kalman_filter.cpp* I have completed implementations of standard Kalman filter and extended Kalman filter, as they share most of the calculations - they were moved to common function UpdateY.
For the EKF, state x is converted to polar coordinates, and then the angle φ is normalized by modulo 2⋅π for *y*.

Throughout the implementation, variables were introduced to save intermediate calculations so they can be reused where necessary.

### 3. Results

After testing filter with simulator I have got these root mean squared errors:

* P<sub>x</sub> = 0.0983
* P<sub>y</sub> = 0.0852
* V<sub>x</sub> = 0.4071
* V<sub>y</sub> = 0.4682

The values satisfy the requirements of the project rubric.

### 4. Discussion

From analyzing the input data, I found out that some measurements are made at relatively small periods down to 0.05 seconds. 
When 2 measurements are made at the same time it's possible to calculate prediction only once as state transition matrix *F* becomes identity matrix
and process noise covariance matrix *Q* becomes zero and values are not updated. However, following this optimization for measurements with *Δt* less than 1/10 of the second has made the result worse.
It means that even small changes are relevant and when simplifying them an error is introduced. 