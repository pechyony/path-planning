[//]: # (Image References)

[image0]: ./images/overview.png "Architecture"
[image1]: ./images/trajectory1.png "Trajectory1"
[image2]: ./images/trajectory2.png "Trajectory2"
[image3]: ./images/trajectory3.png "Trajectory3"
[image4]: ./images/trajectory4.png "Trajectory4"
[image5]: ./images/trajectory5.png "Trajectory5"
[image6]: ./images/fsm.png "fsm"

# Implementation of Path Planner

In this project we implemented a path planner of simulated self-driving car. We used a [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) of term 3 of Udacity Self-Driving Car Nanodegree. This simulator implements Perception, Sensor Fusion, Localization and Motion Control modules of self-driving car.
Our implementation of the path planner consists of 3 modules: 
Prediction, Behavior Planner and Trajectory Generation. The connections between different modules of path planner and simulator are visualized in the following diagram:

![alt text][image0]

Simulator calls the path planner to get a new trajectory for motion controller. Path planner generates trajectory of the car and sends it back to simulator. The trajectory of the car consists of a set of waypoints. In our implementation we generated trajectories with 20 waypoints. Simulator has a motion controller that drives the car from one waypoint to the next one in 0.02 seconds. Apart from the inputs shown in the diagram, path planner receives also the part of the previously generated trajectory that was not used by simulator. This old trajectory is reused by the path planner when generating a new one.

In the next sections we provide a detailed description of Prediction, Behavior Planner and Trajectory Generation modules. 

## Prediction
The goal of Prediction module is to generate an accurate prediction of the location of other cars in the road. Prediction module receives from Sensor Fusion the current location and speed of all observed cars. To generate an accurate prediction of other car's location, we estimated its acceleration by using its two successive observations. We used the following simple formula for estimating acceleration from two successive observations at times t<sub>1</sub> and t<sub>2</sub> (t<sub>1</sub><t<sub>2</sub>): 

<center>
a(t<sub>2</sub>) = (v(t<sub>2</sub>)-v(t<sub>1</sub>))/(t<sub>2</sub>-t<sub>1</sub>)
</center>
<br>
<br>
where v and a are velocity and acceleration. Since Sensor Fusion provides v<sub>x</sub> and v<sub>y</sub> velocities in both x and y axes, we also estimated acceleration a<sub>x</sub> and a<sub>y</sub> separately in each axis. 

Notice that car's acceleration can be estimated only after receiving two observations. Hence we set car's acceleration to zero when it is observed for the first time.

Let t<sub>2</sub> be the time of the last observation of our car. Prediction module predicts locations of other cars at the time when our car reaches the end of the previously unused trajectory: 
<center>
t<sub>3</sub> = t<sub>2</sub> + 0.02 seconds * size of the previously unused trajectory.     
    
</center>
<br>
<br>
Prediction of location and speed to time t<sub>3</sub> is performed using the constant acceleration model. Prediction equations for the x axis are:
<br>
<br>
<center>  
x(t<sub>3</sub>)=x(t<sub>2</sub>)+v<sub>x</sub>(t<sub>2</sub>)dt+a<sub>x</sub>(t<sub>2</sub>)dt<sup>2</sup>/2    

v<sub>x</sub>(t<sub>3</sub>)=v<sub>x</sub>(t<sub>2</sub>)+a<sub>x</sub>(t<sub>2</sub>)dt
</center>

where dt=t<sub>3</sub>-t<sub>2</sub>. Prediction of location and speed along y axis is done in a similar way. 

Prediction module sends generated predictions to Behavor Planner module, which is described in the last section. 

In our code Prediction module is implemented in classes `Car` and `Road`, defined in files `car.h` and `road.h` respectively. The former class represents a single car, while the latter class represents all elements of the road, including all cars in it. Estimation of car's acceleration is implemented in `Car::update` function in `car.cpp` file. Prediction equations are implemented in the function `Car::predict` in `car.cpp` file. 

## Trajectory Generation

Trajectory Generation module generates a smooth trajectory that achieves the desired speed at the desired lane. Our implementation is based on the trajectory generation code described in Q&A video. This code generates trajectories using cubic splines and has the following sequence of steps:      

1. **Compute 4-5 reference points**. We use 3 ways to generate the first 1-2 reference points:  
    Case A. If the previous unused trajectory has at least two waypoints, then we set the first two reference points to be the last two waypoints of the previous unused trajectory.    
    Case B. If the previous unused trajectory has less than two waypoints and the car has non-zero speed then the first two reference points are set to car's location before 0.02 seconds and the car's current location.   
    Case C. If the previous unused trajectory has less than two waypoints and the car has zero speed, we set the car's location to be the first reference point.  

   The next 3 reference points are set in the center of the target lane, 30, 60 and 90 meters ahead of the latest reference point.

   The next diagram shows examples of reference points when the desired lane is the same as the current one (left graph) and when the desired lane is different from the current one (right graph). In both graphs the car moves counterclockwise and triangles are waypoints from the previously unused trajectory.The last two waypoints, used as a reference points for the new trajectory, are marked with blue. The new 3 reference points are shown with blue circles. We will also use this example in subsequent steps of trajectory generation. 

   ![alt text][image1]

2. **Convert the reference points to local coordinate system**. Spline package requires the reference points to have increasing x values. To get this property, in the above Cases A and B we convert all reference points to coordinate system centered at the second reference point and x-axis pointing in the direction from first to second reference point. This transformation is shown in the following diagram:

    ![alt text][image2]

    In Case C we do the same trasformation, but with local coordinate system centered at the first reference point and the local x-axis aligned with the global y-axis. 

3. **Generate a cubic spline that passes through the reference points**. We used a single-header file [spline package](https://github.com/ttk592/spline). To generate a spline we passed a list of reference points to `set_points` function of spline object. The next diagram shows the splines (green lines) generated for our two running examples:

    ![alt text][image3]

4. **Compute new waypoints**. Starting from the second reference point, we compute small segments along a local x axis that can be passed in 0.02 second by driving with the desired speed. The number of such segments is 20 - number of waypoints in the previously unused trajectory. For each endpoint x<sub>i</sub> of the new segments the corresponding y<sub>i</sub> value is the value of the spline at x<sub>i</sub>. _The waypoints from the previously unused trajectory, along with the new points (x<sub>i</sub>,y<sub>i</sub>) are the waypoints of the new trajectory_.

    The following diagram shows 3 new waypoints (top 3 blue triangles) computed along the spline. _These 3 new waypoints along with the other waypoints marked as triangles (both blue and white) are the new trajectory_.

    ![alt text][image4]

5. **Convert new waypoints of a new trajectory to a global coordinate system**. Since simulator assumes that the waypoints are in global coordinate system, we convert the new waypoints (x<sub>i</sub>,y<sub>i</sub>) from local to global coordinate system. The following diagram shows this transformation: 

    ![alt text][image5] 

The entire trajectory generation code can be found in `spline_trajectory` function in `trajectory_generation.cpp` file.

## Behavior Planner

Our Behavior Planner module is a finite state machine with three states: keep lane, change to the left lane and change to the right lane. The following diagram shows all possible transitions between states:

![alt text][image6]

The car starts driving in "keep lane" state. After the first 200 calls to Behavior Planner the car is allowed to change lanes. We introduced this delay for changing the lanes since initially the car drives very slowly and the Sensor Fusion module does not observe all other cars. Under such driving conditions is it very risky to change the lane since other unobserved cars can hit our car from behind. 

To have a smooth driving and to minimize the time the car spends between lanes, the car cannot change a lane immediately after changing a lane. After changing a lane we require a car to spend 200 iterations in "keep lane" state before attempting the next change of lane. 

When there are several candidate states to choose from, we chose the one that has the maximal expected velocity. The computation of the expected velocity depends on the state and is described below:

* **Keep Lane**. In "keep lane" state the expected velocity is the one at the end of the new trajectory. Initially we use predicted locations of other observed cars and check if there will be other car within 30 meters after the end of the previously unused trajectory. If yes, then we generate a new candidate trajectory with the velocity at the last point being the current one decreased by 0.224 mph. If no, then we generate a new candidate trajectory with the velocity at the last point being the current one increased by 0.224 mph, up to a maximal velocity of 49.2 mph. 

* **Change to the left/right lane**. In "change to the right lane" and "change to the left" state the expected velocity depends on the feasiblity of the turn to the new lane. If Prediction module predicts that there will be other car in the new lane within 30 meters after the end of the previously trajectory or within 60 meters before the end of the previously unused trajectory then the change of the lane is not feasible and we set the expected velocity to -1. Otherwise, the expected velocity in the new lane is the average speed of the other cars in the new lane that will drive within 1000 meters of the end of previously unused trajectory when our car reaches that point. If the expected velocity in the new lane is less than the current velocity of the car, even if it slows down a bit due to the car in front of it, then the change to a new lane will not be chosen by behavior planner and hence we don't need to generate a new trajectory. 

    Finally, if it is feasible to change to a new lane and the expected velocity in the new lane is larger than the current velocity then we generate an new candidate trajectory that goes towards the center of the new lane. The target velocity at the last point of the new trajectory is the current velocity plus 0.224 mph, up to a maximal velocity of 49.2 mph.  

Behavior planner is implemented in `behavior_planner.h` and `behavior_planner.cpp` files. 
