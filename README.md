# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program
---
Build a PID controller and tune the PID hyperparameters by applying the general processing flow as described in the previous lessons. Test your solution on the simulator. When the vehicle is able to drive successfully around the track, submit.

In open-loop systems, physical errors may prevent a system, in this case, a self-driving car, from reaching a goal location. The essence of process control is to ensure the system is able to account for those errors as time progresses and ultimately reach the goal location.

PID (Proportional Integral Derivative) is a simple yet effective way of achieving process control.

A PID controller control flow consists of `Error -> PID Controller -> Output`

As the name suggests, a PID controller consists of three parts that all work in different ways to ensure we get to our get:

* Proportional Controller: In this project, the `CTE` represented the cross-track-error, which is the lateral distance from our car to a reference trajectory. The goal of the proportional controller is to steer the car in proportion to the `CTE`. To do this it makes use of a predetermined gain parameter, `Kp`, which helps to maintain this proportionality.
```
Output = Kp * CTE
```
* Integral Controller: In an instance when the wheels of the car are not in alignment, that is when we have a systematic error, our car deviates more from the goal as time progresses. This can be corrected by keeping track of an integral of the `CTE`and multiplying it by a gain constant `Ki`.
```
Output = Ki * Summation(CTE)
```
* Differential Controller: Even after making use of the two controllers mentioned above, we still notice that our car never quite converges at a reference point. It achieves marginal stability as it oscillates around the reference point. To account for this error and to ensure the system, or car, gets to the reference point we keep track of the differential of the `CTE` per unit time and multiply it by a gain constant `Kd`.

``` 
Output = Kd * d(CTE)/dt
```

The PID controller is a summation of the outputs of each individual controller.

## Tuning
I started with initial gains which were not accurate.
```
|         |         |
|---------|---------|
|`Kp`     |0.06     |
|`Ki`     | 0.000001     |
|`Kd`     |1.5   |
```

Making use of the twiddle (Coordinate descent) algorithm implemented in the `twiddle()` function in `helper.h`, I was able to tune those gains based on their RMSE (Root Mean Square Error) gotten from each run of the simulator to obtain
```
|         |         |
|---------|---------|
|`Kp`     |0.285395     |
|`Ki`     | 0.0028573     |
|`Kd`     |3.14895    |

```
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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 