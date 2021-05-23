<h1 align="center">
  <br>
  <a href="https://github.com/VinBots/"><img src="Images/changing_lane.gif" width="278" height="208" alt="DRL"></a>
</h1>

<h4 align="center">2D-Path Planning </h4>
<p align="center">
  <a href="#about">About</a> •
  <a href="#results">Results</a> •
  <a href="#installation">Installation</a> •
  <a href="#configuration">Configuration</a> •
  <a href="#references">References</a> •
  <a href="#credits">Credits</a> •
  <a href="#going-further">Going Further</a>
</p>

---
<h1 align="center">
  <br>
  <a href=""><img src="Images/Screen Shot 2020-08-25 at 14.30.03.png" width="600" alt="DRL"></a>
</h1>
<h5 align="center">Planning and executing trajectories on a highway</h2>


## About

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We are provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

The main files, main.cpp, vehicle.cpp and vehicle.h, are located in the src folder. 
Images folder contain screenshots of a 10 minute simulation (see Performance section)

Note also the use of a really helpful resource for doing this project and creating smooth trajectories: http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use (spline.h)
I used the code provided by Aaron Brown and greatly benefitted from its approach to draw trajectory using the spline library.

main.cpp contains the core functions to use and feed data to the simulator. vehicle.cpp and vehicle.h contain the definition of the class Vehicle that is used to describe the characteristics of the ego car and other cars in the traffic.

The logic of the path planning algorithm has 6 steps:
* data received from the simulator are used to update the status of the ego vehicle (line 276 in main.cpp)
* traffic around the ego car is identified (line 282 in main.cpp) using the function ego.identify_traffic_around_ego(sensor_fusion). This function identifies 1 car by lane, ahead and behind the ego car. Values are stored in a vector of Vehicle objects in ego.vehicles_ahead and ego_vehicles_behind. Note that cars beyond 300m are not considered (perception of an empty lane)
* using a Finite State Machine (FSM) approach, ego.successor_states (line 286 in main.cpp) identifies all the possible future states (without safety considerations).
Note that I introduced a count mechanism in case of a lane change because the transition time to move from one lane to another is much longer than the time between 2 updates of the simulator. It also avoids that the ego changes its mind in the middle of a change lane.
* ego.calculate_best_state(states) (line 289 in main.cpp) calculates the costs associated with each possible state. 2 main factors are taken into account:
    * Safety: changing lane is only possible if there's a minimum safety distance between the ego car and the cars ahead and behind in its lane (see function safety_check in vehicle.cpp)
    * Speed: the fastest lane has the lowest cost. Note that an empty lane has the maximum speed. In this case, the ego car does not wait to be slowed down by a car to change lane (see function best_lane in vehicle.cpp)
* We determine the speed of the car based on the distance to the cars ahead and minimum safety distance.
* Finally, we draw the trajectory of the car (line 310 to 426 in main.cpp), using the spline library to smoothen the trajectory and minimize jerk. Note that in case of a lane change, I augmented the time horizon of the car, by 1s (equivalent to 100 points to send to the simulator) so that the trajectory for the lane change is not re-calculated every 50 points (equivalent to 1s)

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Results

#### Performance

The ego car was able to drive 6.53 miles during 10 minutes (about 39 miles per hour on average) without any collision and by changing lanes whenever it was safe and faster to do so. See images in the folder Images.

#### Reflections and Improvements

Using a finite state machine approach for this project was probably not necessary as the state change could have been simplified by keep lane or change lane but it was a good training.

I was particularly challenged by integrating the time component into the project:
* the simulator uses 1 waypoint every 0.02s but only feeds back the status of the car every 0.06 - 0.08s
* All the state changes do not take the same amount of time (keep lane vs. a lane change)
* instantaneous speed and average speed could be quite different

## Installation

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```
### Dependencies

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
## Configuration

Please see code in `src/main.cpp`

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


## References

* None

## Credits

* Udacity [Flying Car and Autonomous Flight Engineer Nanodegree](https://www.udacity.com/course/flying-car-nanodegree--nd787) program. 

## Going Further

My code could be greatly improved by:
* introducing average speed of lanes based on a moving average of past speeds
* only changing lanes when the increase of speeds is significant
* Safety or braking distance could be more dynamically updated, using the speed of the ego car and other cars. For example, in slow speeds, taking into account the acceleration of the ego car, we could calculate a shorter minimum distance with the car behind.
* Predicting what other cars are doing by analyzing their trajectory.