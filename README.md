# **PID Control**

[//]: # (Image References)

[image1]: ./data/speed1.png "auto-tunning at speed1"
[image2]: ./data/speed2.png "auto-tunning at speed2"
[image3]: ./data/speed1_curv.png "auto-tunning on curv at speed1"
[image4]: ./data/speed2_curv.png "auto-tunning on curv at speed2"

---

[![alt text][video1]](https://youtu.be/AJfq0BIkAko)

---

### Simulator.
This project is a part of Self-Driving Car Engineer Nanodegree Program. You can download the Term 2 simulator which contains the PID controller Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


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
Controller algorithms can vary from simple to complex. Some simple algorithms widely used in industry includes Lead-lag controller, PID controller, etc., [click here for PID controller](https://en.wikipedia.org/wiki/PID_controller). The simple controller has a linear equation to output an actuation signal that is in proportion to the amount of error(p-gain), derivatives of error(d-gain), and total error(i-gain). Characteristics of P,I,D are shown as below.

|closed loop response| rise time    | overshoot | settling time | steady state error |
|--------------------|--------------|-----------|---------------|--------------------|
|increase p gain     | Decrease     | Increase  | Small change  | Decrease           |
|increase i gain     | Decrease     | Increase  | Increase      | Eliminate          |
|inccreas d gain     | small change | Decrease  | Decrease.     | Small change       |

[check out what is rise time, overshoot, settling time, steady state error](https://ni.scene7.com/is/image/ni/12fbdcae1636?scl=1)


2. **Automatic PID tunning**

PID parameter tunning depends on the characteristics of system. And it is known that there's no 'one-size-fit-all' tunning method. For the project, i decided to go for a model-based auto-tunning using our python script of kinematic bicycle model. I modified it to write the auto-tunning script. [click here for the script](./PID_auto-tunning.ipynb). In conclusion, the P,D,I gain found for steering is [0.15910442248556678, 1.587110426670305, 0.005533208544239475].

    - **Coordinate Ascent**

Coordinate ascent is an optimization technique where each dimension is maximized(exact or inexactly), with other dimensions fixed. The local search method can be used with or without gradients but its performance heavily depends on initialization. 
So, i first decided a promising initial values of p-gain with trials and then optimized it using CA with d-,i-gain fixed 0. Once the p-gain was optimized, i fixed the p-gain and run CA again to optimize d-gain with i-gain fixed 0. And once d-gain was optimized, i fixed the d-gain and run CA again to optimize i-gain. After each gain was optimized, i ran CA again altogether with the 3 fixed values. For each gain, i set the CA tolerance value to be sufficiently small enough(.002) to make sure that it converges enough to a local minima.

Note that there were 2 different optimizations. One with P,I,D saught at speed = 1 and the other at speed = 2. Depending on the speed fed into our kinematic model, a PID set was different. Both PID sets passed the test at 50mph.

![alt text][image1]
Speed 1 params=[0.3083945237000776, 3.73066396507075, 0.009287418277686622]

![alt text][image2]
Speed 2 params=[0.15910442248556678, 1.587110426670305, 0.005533208544239475]


    - **P,I,D at Speed = 2**
    
I chose the PID set saught at speed = 2 for the simulation. The reason was partly because the delta time that the websocket messages arrive in our C++ program was greater than 20msec(our simulator cycle). Remember from the previous path-planning project that our simulator cycle was 0.02 sec, and we were tossed back from the simulator around 50~70% left-over moves(out of 50) that were not eaten by our simulator. Term 2 simulator would not take that long but it differ depend on system performance. Just to be safe, i assumed 80msec. And this means i could enforce my desired speed in around 80msec interval, 12 moves per second. And since our desired speed was around 50mph(25m/sec, 0.5m per move), the speed here was set 2m per move to achieve it in 12 moves.

The other reason was it showed less error on a curve, especially around 50mph, as shown below(although it showed more error on a line trajectory). It showed the Final Error(0.00198) over the speed = 1 case(0.01354). The figures below shows the comparison.

![alt text][image3]
Speed 1 on curve params=[0.20877822643745259, 3.0511517488783064, 0.005540710267815578]


![alt text][image4]
Speed 2 on curve paramms=[0.15910442248556678, 1.587110426670305, 0.005533208544239475]

3. **Longitudinal Speed Controller with PID**

For the longitudinal speed control, i set P,D,I gain manually. They were set to [0.05, 0.5, 0.] with bias 0.1. As the speed is a feedback from the dynamic system of car, the speed error to the desired speed setpoint(here, 50mph) is input to the longitudinal PID speed controller. Note that i set throttle value proportional to the magnitude of derivative of steering. The intention behind such setting was that the sharper it steered, the harder it stepped on the break.

```
throttle_value = 0.05*(50-speed) - 0.5*fabs(steer_value) + 0.1;
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
