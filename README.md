# Udacity Self-Driving Car Nanodegree
## Path Planning Project

### Introduction
The goal of this project is to navigate the car around a simulated highway with traffic in multiple lanes using waypoint, telemetry, and sensor fusion data. The car must drive on the highway with motion constrains and goals defined below:
* Follow posted speed limit - 50mph
* Avoid collisions
* Stay in lane or change lanes when its safe
* Maintain speed or slow down based on traffic condition
* Minimize jerk and calculate smooth trajectories

### Data Processing
#### Inputs
The simulator provides the following inputs:
* Localization data
  * (x, y) co-ordinates
  * (s, d) Frenet co-ordinates
  * Yaw rate
  * Speed

* Sensor fusion data, that provides Localization of other vehicles
  * Lane of the vehicle (defined as "d")
  * Velocity defined by vx and vy
  * Speed (Magnitude) determined using equation sqrt(vx**vx+vy**vy)
  * How close the vehicle is to our self driving car, which is determined by "s" value

#### Outputs
* The output of the program should be a list of co-ordinates determined as (x, y) points, that indicate the next desired positions of the car. The duration of movement from one point to another is defined as 20 milliseconds.
* Determine actions like lane changes(left/right) and maintain speed or slow down using sensor fusion data

#### Process Motion
The basic motion to stay in the same lane is achieved by incrementing the "s" value in Frenet co-ordinates and keep "d" value constant. We generate 50 way points that determine next location with each reached in a time step of 20 milliseconds. To generate smooth transition and minimize `jerk` previous path points are used to avoid any discontinuity and a `spline` polynomial is created using calculated waypoints what are 30 meters, which will also be the reaction distance to slow down for the cars in front of the self driving car. The spline C++ library used is available at https://github.com/ttk592/spline. The steps to create spline are given below:

1. Generate desired points as (x, y) co-ordinates. The source snippet defined below helps achieve this goal.
```
//Spline Steps
static const vector<int> spline_steps = {30,60,90};
for(int i = 0; i<spline_steps.size(); i++) {
    vector<double> next_wp = getXY(car_s+spline_steps[i], (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp[0]);
    ptsy.push_back(next_wp[1]);
}
```
Each lane is assumed to be 4 meters wide. The getXY returns transformed (x, y) co-ordinates. The s values are determined in 30, 60 and 90m steps and d position is determined based on the desired lane car needs to drive on.

2. Creating a spline polynomial is done using the following piece of code:
```
tk::spline s;
s.set_points(pts_x, pts_y);
```
3. The y-coordinate is determined using the simple assignment below:
```
double y_point = s(x_point);
```
4. To achieve smooth running car we fill up the rest of our path planner after re-using previous path points. The output is always 50 points. The following code snippet describes this approach.
```
for(int i = 1; i <= 50-previous_path_x.size(); i++) {
    double N = (target_dist/(0.02*ref_vel/2.24)); //Change distance to metres per seconds.
    double x_point = x_add_on+(target_x)/N; //Find x_point using initial point x_add_on and target_x/N(Number of hash points)
    double y_point = s(x_point);

    x_add_on = x_point;
    double x_ref = x_point;
    double y_ref = y_point;

    //Rotate back to normal after rotating it earlier
    x_point = (x_ref *cos(ref_yaw)-y_ref *sin(ref_yaw));
    y_point = (x_ref *sin(ref_yaw)+y_ref *cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
}
```

5. Finally the points are transformed into map co-ordinate system using the transformation detailed below:
```
//Rotate back to normal after rotating it earlier
x_point = (x_ref *cos(ref_yaw)-y_ref *sin(ref_yaw));
y_point = (x_ref *sin(ref_yaw)+y_ref *cos(ref_yaw));
x_point += ref_x;
y_point += ref_y;
next_x_vals.push_back(x_point);
next_y_vals.push_back(y_point);
```

#### General Safety
The general safety aspects implemented as part of this project are
* Collision Avoidance (By Accelerating or Decelerating)
* Lane Changes
* Maintain Steady Speed

The implementation follows a simple state machine described below:
1. Define the states as an enumeration below:
```
//Define Finite State Machine states
enum State {
    STAY_IN_LANE,
    SLOW_DOWN,
    SWITCH_LEFT,
    SWITCH_RIGHT,
    MAINTAIN_SPEED
};
```
2. Process sensor fusion data to determine location and speeds of cars around the self driving car simulated as follows:
```
for(int i=0;i < sensor_fusion.size();i++) {
    //Find car is in which lane. The "d" value of each car helps us if its in our lane or other lanes.
    float d = sensor_fusion[i][6];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx+vy*vy); //Magnitude
    double check_car_s = sensor_fusion[i][5]; //S coordinate value identifies if its close or not

    //Projecting into the Future (Next Frame)
    check_car_s+=((double)prev_size*0.02*check_speed); // if using previous points it can project s value output
    double s_distance = check_car_s - car_s;

    //With each lane being 4 metres, we idenfify if its between 4 and 8 "6" being middle lane
    //Check speed of the car if its in current lane
    if(is_on_lane(d, lane)) { //Same lane
      //Check s values greater than our car and s gap
      //Check if the car is too close or not
      if((s_distance > 0) && (s_distance<detection_distance)) {
          //Lower reference velocity to avoid crash or collision with cars in front
          //Set flag for lane change as well
          //Get the speed of car in front
          too_close = true;
          car_in_front_speed = check_speed *2.24;
      }
    } else if (is_on_lane(d, lane-1)) { //Left lane
              if(s_distance > -detection_distance_back && s_distance < detection_distance) {
                  car_on_left_lane = true;
              }
    } else if (is_on_lane(d, lane+1)) { //Right lane
              if(s_distance > -detection_distance_back && s_distance < detection_distance) {
                  car_on_right_lane = true;
              }
    }
}
```
3. After finding the location and speed of cars around our self driving vehicle, we set the desired target state as follows:
```
//Lane and maneuver decisions
if (too_close) {
    if(lane > 0 && !car_on_left_lane) { //Not on Left Lane and there are no cars blocking
      state = SWITCH_LEFT;
    } else if (lane < 2 && !car_on_right_lane) { //Not on Right Lane and there are no cars
      state = SWITCH_RIGHT;
    } else if(ref_vel > car_in_front_speed) { //If the current speed is more than estimated speed of car in front
      state = SLOW_DOWN;
    } else {
      state = MAINTAIN_SPEED;
    }
} else {
  state = STAY_IN_LANE;
}
```

4. Finally executed action determined in the previous step as follows using a switch...case statement below:
```
//Set maneuvers - change lanes, slow down or maintain speed based on the desired state set in the previous step
switch(state){
    case SWITCH_LEFT: lane--; break;
    case SWITCH_RIGHT: lane++; break;
    case STAY_IN_LANE:
        if(ref_vel < max_vel){
          //ref_vel += 7 / .224*time_delta;
          ref_vel += 0.224;
        }
        break;
    case SLOW_DOWN:
      //ref_vel -= 4 / .224*time_delta;
        ref_vel -= 0.224;
        break;
    case MAINTAIN_SPEED:
        break;
    default:
        break;
}
```

### Conclusion
The tests on simulator have been incident free after driving for many miles. Although there are other approaches like using Quintic polynomial, the results of using Spline library performance has been found to be optimal performance on the highway with less code. Trying to research if any best practices could be used from approaches like Hybrid A*. Overall, the results have been good.

---

### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. && make`
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
