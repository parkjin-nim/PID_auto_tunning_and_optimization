# **PID Control**

[//]: # (Image References)


[image0]: ./data/43mph.png "running_at_43mph"
[image1]: ./data/speed1.png "auto-tunning at speed1"
[image2]: ./data/speed2.png "auto-tunning at speed2"
[image3]: ./data/speed1_curv.png "auto-tunning on curv at speed1"
[image4]: ./data/speed2_curv.png "auto-tunning on curv at speed2"

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

[check out rise time/overshoot/settling time/steady state error](https://ni.scene7.com/is/image/ni/12fbdcae1636?scl=1)



2. **Automatic PID tunning**

PID parameter tunning depends on the characteristics of system. And it is known that there's no 'one-size-fit-all' tunning method. For the project, i decided to go for a model-based auto-tunning using our python script of kinematic bicycle model. I modified it to write the auto-tunning script. [Check out above pid_auto-tunning.ipynb file](./PID_auto-tunning.ipynb). 

- **Coordinate Ascent**

Coordinate ascent is an optimization technique where each dimension(coordinate) is maximized(exact or inexactly), with other dimensions fixed. The local search method can be used with or without gradients but its performance heavily depends on an initialization. 

So, i first decided a promising initial value of p-gain and then optimized based on it using CA with d-,i-gain fixed 0. Once the p-gain was optimized, i fixed the p-gain and run CA again to optimize d-gain with i-gain fixed 0. And once d-gain was optimized, i fixed the d-gain and run CA again to optimize i-gain. After each gain was optimized, i ran CA again altogether based on the 3 fixed values. For each gain, i set the CA tolerance value to be sufficiently small enough(.002) to make sure that it converges enough to a local minima.

Note that there were 2 different optimizations. One with P,I,D saught at speed = 1 and the other at speed = 2. Depending on the speed fed into our kinematic model, a PID set was different but each PID sets passed the simulation test at 50mph in a different driving pattern.

Below is the system responses of P, I, D, PID auto-tunning at Speed=1 with params=[0.3083945237000776, 3.73066396507075, 0.009287418277686622]

![alt text][image1]

Below is the system responses of P, I, D, PID auto-tunning at Speed=2 params=[0.15910442248556678, 1.587110426670305, 0.005533208544239475]


![alt text][image2]


- **P,I,D at Speed = 2**
    
I chose the PID set saught at speed = 2 for the simulation. The reason was partly because the delta time that the websocket messages arrived in our C++ program was higher than 20msec(our simulator cycle). Remember from the previous path-planning project that our simulator cycle was 0.02 sec, and we were tossed back from the simulator around 50~70% left-over moves(out of 50) that were not eaten by our simulator. Term 2 simulator would not take that long but it should differ depending on a system. Just to be safe, i assumed 80msec. And this meant i could enforce my desired speed on 12 moves per second. And since our desired speed was around 50mph(25m/sec, 0.5m per move), the speed here was set 2m per move to achieve it in 12 moves.

The other reason was it showed less errors on a curvy trajectory, especially at around 50mph, as shown below. Although it showed more error on a line trajectory, tt showed the Final Error(0.00198) over the speed = 1 case(0.01354). The figures below shows the comparison.

Below is the system responses of P, I, D, PID auto-tunning at Speed=1 on a curve with params=[0.20877822643745259, 3.0511517488783064, 0.005540710267815578]
![alt text][image3]


Below is the system responses of P, I, D, PID auto-tunning at Speed=2 on a curve with paramms=[0.15910442248556678, 1.587110426670305, 0.005533208544239475]

![alt text][image4]


3. **Longitudinal Speed Controller with PID**

For the longitudinal speed control, i set P,D,I gain manually. They were set to [0.05, 0.5, 0.] with bias 0.1. As the speed is a feedback from the dynamic system of car, the speed error to the desired speed setpoint(here, 50mph) is input to the longitudinal PID speed controller. Note that i set throttle value proportional to the magnitude of derivative of steering. So, the sharper it steered, the harder it stepped on the break.

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
