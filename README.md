# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Project writeup
1) The code compiles correctly.  
> Yes, it does. See [this](https://github.com/smashkoala/CarND-Path-Planning-Project) repository in GitHub.

2) The car is able to drive at least 4.32 miles without incident.  
> Yes, the car is able to drive longer than 4.32 miles. The video clip is [here](https://github.com/smashkoala/CarND-Path-Planning-Project/blob/master/4_3Miles.mp4) in GitHub.

3) There is a reflection on how to generate paths.
> Here, I will explain how the path is generated in the code.
The main() in main.cpp consists of mainly four parts.  
(1) Possible path prediction (line 431-475 of main() in main.cpp)  
(2)Cost calculation for each path(471-473 of main(), calculate_cost_pos() and calculate_cost() line 195-251 in main.cpp)  
(3) Path decision making (line 478 of main(), decide_next_action() line 273-290 in main.cpp)  
(4) Trajectory generation.(line 499-570 in main.cpp)

>3 x 3 actions (Lane change: Keep, left, right, speed change: Keep, speed up, speed down) are used to predict the possible paths. For each action combination, the cost is calculated. It chooses the lowest cost action combinations. In the process of (4), based on the chosen lane, the next waypoints of 30m, 60m and 90m are calculated. Using spline.h and these waypoints, an ideal trajectory is calculated. The chosen speed is then used to calculate the next 30m path and points that car actually follows every 0.2s using the ideal trajectory.

4) The car drives according to the speed limit.  
> In the code, the target speed is set 47 MPH, and it does NOT exceed the speed limit of 50 MPH.
In the cost calculation function, the actions that exceeds over 47MPH are severely penalized. (calculate_cost() line 222-250 in main.cpp)

5) Max Acceleration and Jerk are not Exceeded.  
> It does not exceed the limits of max acceleration and jerk.
The acceleration is adjusted by some experiments. When the car starts moving at the begging,
it is expected to accelerate 2.0 MPH during 0.02 seconds, which is 2.0 * 1609.34 / (3600 * 0.02) = 44.07 m/s2. This does not mean that the car accelerates 44.07 m/s2 since this value is used to generate the trajectories of future path. It was set this way, since it takes too much time for the car to speed up if the acceleration is set less than 10 m/s2 at the begging.
When the car speed becomes faster than 30 MPH, the acceleration is set 0.8 MPH, which is 0.8 * 1609.34 / (3600 * 0.02) = 17.9 m /s2. The deceleration is the same with minus value.
With these settings, after some experiments, it was confirmed that the max acceleration and the jerk did not exceed the limit. (line 484-496 in main.cpp)

6) Car does not have collisions.  
> It does not have collisions. The code checks if others cars are within plus minus 30 meter. If they are, the cost of the chosen actions are penalized with higher cost values. (line 195-219 in main.cpp)

7) The car stays in its lane, except for the time between changing lanes.  
> It does stay in its lane except lane changes. The future trajectories are drawn based on which lanes to go. Once the lane to drive is chosen, a continuous trajectory is drawn for the next 30, 60 and 90m on the Frenet coordinate. The car follows this trajectory, and it does not stop in the middle of the lane. (line 563-565 in main.cpp)

8) The car is able to change lanes  
> When other cars are ahead of the Ego car, and the distance between other cars and the Ego car gets closer, if no other car stays on either of left or right lanes, and if it has clearance of plus minus 30 meter in either left or right lane, it changes the lane based on the calculated costs of lane change vs staying in current lane. (calculate_cost_pos() line 195-218 in main.cpp)


### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
