# Writeup
Here follows a description of how each of the rubrics required for the **Building an Estimator project** were achieved.<p>
This project is part of the Udacity Flying Car and Autonomous Flight Engineer Nanodegree Program

## Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.

## Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.
Using the provided Quaternion class we can create a quaternion based on the available estimates for roll, pitch and yaw. The Quaternion class also provides a method for integration.
```
Quaternion<float> quaternion = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
quaternion.IntegrateBodyRate(gyro, dtIMU);
```
After integration, new estimates for roll, pitch and yaw are available.
```
float predictedPitch = quaternion.Pitch();
float predictedRoll = quaternion.Roll();
ekfState(6) = quaternion.Yaw();
```
## Implement all of the elements of the prediction step for the estimator.
First we need to convert the acceleration measurements from the body frame to the inertial frame
```
V3F accelWorld = attitude.Rotate_BtoI(accel);
```
Then the state matrix can be updated with new estimates of roll, pitch, yaw, roll rate, pitch rate and yaw rate.
```
predictedState(0) += predictedState(3) * dt;
predictedState(1) += predictedState(4) * dt;
predictedState(2) += predictedState(5) * dt;
predictedState(3) += accelWorld.x * dt;
predictedState(4) += accelWorld.y * dt;
predictedState(5) += accelWorld.z * dt - CONST_GRAVITY * dt;
```
Formulas come from Lesson 2 - Chapter 12: Dead Reckoning 3D

## Implement the magnetometer update.
First we calculate the yaw error by comparison of the current estimate and the magnetometer measurement
```
zFromX(0) = ekfState(6);
float yawErr = z(0) - zFromX(0);
```
We then update the first item of the measurement model matrix without forgetting to normalize the yaw between -Pi and Pi
```  
if (yawErr > F_PI) z(0) -= 2.f * F_PI;
if (yawErr < -F_PI) z(0) += 2.f * F_PI;
```
hPrime is simply a zeros matrix with the last item equal to 1
```
hPrime(0,6) = 1;
```
Formulas come from item 7.3.2 from the Estimation for Quadrotors paper

## Implement the GPS update.
We update the measurement model matrix composed of the three angles and their first order derivatives
```
zFromX(0) = ekfState(0);
zFromX(1) = ekfState(1);
zFromX(2) = ekfState(2);
zFromX(3) = ekfState(3);
zFromX(4) = ekfState(4);
zFromX(5) = ekfState(5);
```
hPrime is simply an identity matrix
```
hPrime(0,0) = 1;
hPrime(1,1) = 1;
hPrime(2,2) = 1;
hPrime(3,3) = 1;
hPrime(4,4) = 1;
hPrime(5,5) = 1;
```
Formulas come from item 7.3.1 from the Estimation for Quadrotors paper