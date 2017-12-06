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
