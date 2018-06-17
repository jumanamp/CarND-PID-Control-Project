# CarND-Controls-PID
This repository contains the project on implementation of a PID Controller,
for controlling the motion of an autonomous car around a circular track simulated on
a simulator, as a part of Term 2 of Self Driving Car Nano degree by Udacity. The PID
controller helps to derive steering angles for directing the motion of
the car along the track as smooth and centred as possible. The code for the project
is written in C++.

---

## Introduction
PID controller is universally accepted and most commonly used controller in industrial application because PID controller is simple, provide good stability and rapid response.
PID controller is a Closed loop control system which has feedback control system and it compares the Process variable (feedback variable) with set Point and generates an error signal and according to that it adjusts the output of system. This process continues until this error gets to Zero or process variable value becomes equal to set point.

PID stands for proportional, integral, derivative. In each application, coefficient of these three actions are varied to get optimal response and control. Controller input is error signal and output is given to the plant/process. Output signal of controller is generated, in such a way that, output of plant is try to achieve desired value.


### PID Controller Basics

PID controller is a combination of three terms; Proportional, Integral and Derivative. These
components are described below.

* **Proportional(P) Response:** The proportional control mode is in most cases the main driving force in a controller. It changes the controller output in proportion to the error P component of the control output is large, when the error is large and small otherwise. The mathematical relationship is expressed as:

  `P = -Kp * error, where Kp is the gain factor.`

  The adjustable setting for proportional control is called the Controller Gain (Kp). A higher controller gain will increase the amount of proportional control action for a given error. If the controller gain is set too high the control loop will begin oscillating and become unstable. If the controller gain is set too low, it will not respond adequately to disturbances or set point changes.

* **Derivative(D) Response:**

  Term ‘D’ is proportional to the rate of change of the error. The mathematical relationship is expressed as:

  `D = -Kd * d(error)/dt, where Kd is the gain factor.`

  Output of derivative controller is directly proportional to the rate of change of error with respect to time as shown in equation.  Generally, Derivative controller is used when processor variables starts oscillating or changes at a very high rate of speed. D-controller is also used to anticipate the future behaviour of the error by error curve.

* **Integral(I) Response:**

  Term ‘I’ is proportional to the average value of the error. The mathematical relationship is expressed as:

  `I = -Ki * sum(error), where Ki is the gain factor.`

  Integral controller is generally used to decrease the steady state error. Term ‘I’ is integrate (with respect to time) to the actual value of the error. Because of integration, very small value of error, results very high integral response. Integral controller action continues to change until error becomes zero.

For a PID controller, all the three components are used with different weights form each decided
by the value for each of their gain factors. Note that the proportional gain effects
response from other components. Mathematical equation for a PID controller is given by:

```
$\alpha$ = P + I + D
       where, $\alpha$ is the control input to the system, known as the **actuator** input.
```
---

## Project Details

### Goal
In the project, a PID controller was used to drive an autonomous car around a circular track
with sharp left and right turns, by using the controller to derive steering angle. The major challenge is to make sure that the car drives smoothly following the centre of the lane as well
as take smooth turns in the edges of turns.

[Udacity's self driving car simulator](https://github.com/udacity/self-driving-car-sim/releases) for this was provided as a part of the course. Simulator measures the cross track error(**cte**), the error between the lateral position of the car and the centre of the track. This value is received by the project code using the  [uWebSockets library](https://github.com/uNetworking/uWebSockets) library, we used as part of the project. **cte** is the actuator input we use as actuator input to the PID controller, which then directs the steering angle of the car.


### Steps Taken

The implemented solution for the project mainly consists of the following major steps:

1. Implementation of the PID controller used the cross track error, **cte** from the simulator
  leading to the PID controller equation:
  ```
  $\alpha$ =  - (Kp * cte) - (Ki * \sum(cte)) - (Kd * d(cte)/dt)
  ```

2. Major task for the project involved tuning the gain parameters *Kp*, *Ki* and *Kd* until the car drives smoothly around the track. This tuning step was performed manually by trial and error approach.

3.  Trial and Error Approach Description:
  * **P component:**

    P component has the most direct impact on the motion of the vehicle, as it is directly
    proportional to the cross track error. I started the trial approach with small value around 0.1 and kept increasing it, without other components. As expected, with higher values, the car seems to speedup when the error is large causing it overshoot track quickly, especially at turns. I reduced the constant and finally decided to settle to a value between 0.5 and 2 after adding other controllers and moved on to use derivative controller, to prevent such oscillations to bring the car speed under control when taking turns.

  * **D component:**

    Using derivative controller indeed brought the car under control in turns,
    taking smooth turns in sharp turns. I started with higher values at around 3 and kept reducing them, as I found that with higher *Kd* value, though car recovers quickly from not overshooting around edges, it still oscillates around the centre lane more lie a paranoid driver than moving smoothly. I finally settled down with a value of 2 and *Kp* value of 0.2.

  * **I component:**  

   Using integral controller helps to correct any systematic bias introduced in the error while car is tracking the centre of the lane and a constant offset is introduced. It is proportional to the constantly incremented sum of error values. In our case, such a systematic bias does not seem to occur and hence this value can be very low. Also I introduced a ring buffer to save only last 100 error values to be integrated. After some experiments around this, I settled for a final value of *Ki* as 0.0001.

---
## Project Outcome

### Project Code and Build instructions
#### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

#### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
