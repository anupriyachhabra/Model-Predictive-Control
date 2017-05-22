# CarND-Controls-MPC
This project is a part of Udacity's Self-Driving Car Engineer Nanodegree Program. Goal of this project is to drive a car
autonomously around the track in a simulator using a Model Predictive Controller.


## The Model
The Model used in this project is a simple Kinematic Model consisting of Vehicle State and Actuators. 
Vehicle State refers to the state of the vehicle and can be represented as a vector (px, py, psi, v, cte, epsi). 
px = vehicle’s x coordinate, py = vehicle’s y coordinate , psi = vehicle’s orientation or heading direction, v = vehicles velocity, 
cte = cross track error (distance of vehicle from expected trajectory) and epsi = error in orientation of vehicle compared to reference trajectory.
Actuators are set of controls used to navigate the vehicle. 
For simplicity this project only uses 2 actuators which are represented as vector (delta, a). 
delta stands for next steering angle that vehicle should apply, 
in real world a vehicle cannot take very sharp turns due to vehicles turn radius, 
this has been accounted for in the model constraints and vehicle cannot turn more than +/-18 degree (+/- 0.33 radians) 
in this simulated environment. 

a referes to acceleration or throttle that should be applied to vehicle. 
It can have both positive values for speeding and negative values for breaking.

The Kinematic equations used for updating vehicle state are as follows  
x​t+1​​=x​t​​+v​t​​∗cos(ψ​t​​)∗dt  
y​t+1​​=y​t​​+v​t​​∗sin(ψ​t​​)∗dt  
ψ​t+1​​=ψ​t​​+​L​f​​​​v​t​​​​∗δ∗dt  
v​t+1​​=v​t​​+a​t​​∗dt  
cte​t+1​​=f(x​t​​)−y​t​​+(v​t​​∗sin(eψ​t​​)∗dt)  
eψ​t+1​​=eψ​t​​+​L​f​​​​v​t​​​​∗δ​t​​∗dt  


## Timestamp length and Frequency
This model predicts vehciles state after every 100ms which is represented by dt in the kinematic equations above. 
100 ms was chosen as taking a value higher than this was giving inaccurate results as the environment was changing too fast 
for the predictive controller to give accurate results. Changing to a value lower than 100ms was also of not much use as 
we were unnecessarily making computations for 2 very similar states.

Also the model only predicts next 10 states which is represented by variable N in `mpc.cpp`. I tried higher values of N 
such as 20 but this burdened the ipopt solver used in this project to not produce optimal results as I had constrained 
the ipopt solver to compute in less than 0.05 seconds.

This N multiplied by dt (0.1 *10) gives 1 second, so our predictive controller only predicts the set of states for next 1 second. 
This seems about correct for a highly volatile environment for a vehicle driving on a high speed in a high traffic environment.


## Polynomial Fitting and MPC Preprocessing
I have used polynomial fitting of 3rd order on the waypoints which are returned by simulator and used this polynomial to 
evaluate a reference trajectory for the vehicle path. This reference trajectory is represented by yellow line in the 
model [simulation](#simulation).

Also I have pre-processed the way points and converted them to vehicle coordinates as mentioned in project tips and tricks.
This was an important part of the project as this helped with easier debugging of vehicle predicted track in simulator.


## Handling Latency

This project also factors in real world latency that can occur while applying actuator inputs. To simulate this the 
project's main thread sleeps for 100ms before sending the actuations to simulator. To account for this while returning the 
actuations to simulator I use 2 set of actuations — the real actuations for next step and the next predicted actuation 
after dt which is 0.1 second(100 ms) in my case. Sending the sum of these 2 actuations makes the model proactively apply 
the next actuation and hence handles the 100ms latency.


## Simulation

Here's a [link to my video result](https://youtu.be/ERkIpLQWWK0)


---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
