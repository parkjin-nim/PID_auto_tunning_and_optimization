# **PID Control**

[//]: # (Image References)


[image0]: ./data/43mph.png "running_at_43mph"
[image1]: ./data/rise_time.jpeg "rise time, overshoot, settling time, steady state error"
[image2]: ./data/speed1.png "auto-tunning at speed1"
[image3]: ./data/speed1_curv.png "auto-tunning on curv at speed1"


---

![alt text][image0]

---

### Simulator.
This project is a part of Self-Driving Car Engineer Nanodegree Program. You can download the Term 2 simulator which contains the PID controller Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals

In this project the goal was to implement a PID controller in C++ to maneuver the vehicle around the track in the simulator. The simulator provided the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle. The car would try to go as close as possible to the 50 MPH speed limit. Here, the PID gains were selected such that the car was close to the desired speed(50mph) and had the least errors on the curvy trajectories.

Auto-tunning was the major tool to achieve the goal. The coordinate ascent optimization was utilized to fit the PID parameters automatically. 


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Here is the data provided from the Simulator to the C++ Program

#### PID controller input

["cte"] Cross-track error

["speed"] The car's speed in MPH

["steering angle"] The car's steering angle.


Here is the data provided from the C++ Program to the Simulator.

#### PID controller output

["steering angle"] The car's steering angle[-1,1].

["throttle"] Throttle percentage[0,1].



## Details

1. P,I,D controller

Controller algorithms can vary from simple to complex. Some simple algorithms widely used in industry include Lead-lag controller, [PID controller](https://en.wikipedia.org/wiki/PID_controller). The simple controller has a linear equation to output actuation signals that are in proportion to the amount of error, derivative of error, and total error. Characteristics of P,I,D are shown as below.

|closed loop response| rise time    | overshoot | settling time | steady state error |
|--------------------|--------------|-----------|---------------|--------------------|
|increase p gain     | Decrease     | Increase  | Small change  | Decrease           |
|increase i gain     | Decrease     | Increase  | Increase      | Eliminate          |
|inccreas d gain     | small change | Decrease  | Decrease.     | Small change       |


Here's the concept of rise time, overshoot, settling time, and steady state error.

![alt text][image1]


2. **Automatic PID tunning**

PID parameter tunning depends on the characteristics of system. And it is known that there's no 'one-size-fit-all' tunning method. For the project, i decided to go for a model-based auto-tunning using our python script of kinematic bicycle model. I modified it to write [my auto-tunning script](./PID_auto-tunning.ipynb). 

Coordinate ascent is an optimization technique where each dimension(coordinate) is maximized(exact or inexactly), with other dimensions fixed. 

```
p, s = [0, 0, 0], 1
tol = 0.00002

dp = [1, 0, 0]
it,params, err = twiddle(tol, p, dp, s)
robot = make_robot()
_, _, err = run(robot, params, speed=s)
init_p = params[0]

# optimize d with others fixed.
p = [init_p, 0, 0]
dp = [0, 1, 0]
it,params, err = twiddle(tol, p, dp, s)
robot = make_robot()
_, _, err = run(robot, params, speed=s)
init_d = params[1]

# optimize i with others fixed.
p = [init_p, init_d, 0]
dp = [0, 0, 1]
it,params, err = twiddle(tol, p, dp, s)
robot = make_robot()
_, _, err = run(robot, params, speed=s)
init_i = params[2]
```


So, i initially set parameters all zeros and optimized one by one. First p-gain was optimized and fixed, then d-gain was optimized with i-gain set to 0. Then d-gain was fixed and i-gain was optimized. In the process, i found the tolerance value of coordinate descent because it has an impact on the settling time. 

![alt text][image2]

This way the P,I,D gain was found as below.

```
params=[0.2864386636430725, 3.0843418153144158, 0.01033423736942282]
```

Since the parameters were set and tested only for the line trajectory, i further fine-tunned the parameters to embrace curve trajectory. 

![alt text][image3]

And this way, the final parameters for steering was found as below.

```
params=[0.3063393801031459, 3.757411395133915, 0.00895626708915423]
```

3. **Longitudinal Speed Controller with PID**

For the longitudinal speed control with PID, i used the speed feedback to calculate the error. Here, i set the desired speed setpoint 50mph. And then i used the steering value calculated to slow down the throttle. So, the sharper turn it made, the harder it stepped on the break. Then there was throttle bias, 0.1.

```
throttle_value = 0.01*fabs(50-speed) - 0.5*(fabs(steer_value)) + 0.1;
```
---

## Dependencies

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).
