# **Project 4 Submission in CarND-Term2**
This repository contains project 4 submission in Self-Driving Car Nano-Degree, Term 2, PID controller. Modified source codes are all included in folder ./src. The following files are modified:

##### 1. PID.cpp
##### 2. PID.h
##### 3. main.cpp

##### 1. The effect of the P, I, D component of the PID algorithm
a) P component is proportional to the instantaneous cross track error (CTE), and it tries to steer the wheels to the opposite direction of CTE and drive the car back to track. However, P component will lead the car to keep crossing the center track and cause overshoot. P parameter can be turned by watching overshoot and settling time.

b) I component is proportional to the integration of CTE, and it is used to correct any bias at the steering function. Think about in a steady state, steering system bias will cause certain offset without proper I component. I parameter can be tuned by watching the steady state error.

c) D component is proportional to the derivative of CTE, and it is used to introduce damping function to reduce overshoot. D parameter can be tune by watching the overshoot.

##### 2. hyper-parameters tuning

PID hyper-parameters are first tuned manually to make sure the car can roughly follow the track. Then a twiddle algorithm is implemented to fine tune the parameters by comparing the total CTE error. I borrowed a few techniques from the [the post](https://github.com/jeremy-shannon/CarND-PID-Control-Project/blob/master/README.md) to make the twiddle working, such as using settle_steps and eval_steps.

I implemented one PID controller for steering, and one PD controller for throttle. Either controller can enable twiddle feature independently. In really, I ran twiddle for steering controller first while keeping throttle controller with fixed parameters, and then ran twiddle for throttle controller while keeping steering controller with parameters found previously. The following parameters were found out after these two trial:

Steering PID controller: `P = 0.140937; I = 0.000302802; D = 3.05879`

Throttle PID controller: `P = 0.33344; I = 0.0; D = 0.02004`;

In order to prevent the speed dropping to zero at some condition, a `low_speed_limit` was set, whenever the speed is lower then this limit, the throttle will be set the a default value and the car is pushed forward. In order to smooth the throttle value when the speed is crossing `low_speed_limit`, a weighting method is introduced to the throttle controller, as below:

 `throttle_value = low_speed_limit/speed * default_throttle_value + (1.0 - low_speed_limit/speed) * PID_value`

When speed is close to `low_speed_limit`, the default throttle value will have more weight, when the speed is high enough, the value from PID control is dominant.
