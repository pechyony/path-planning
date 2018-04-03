[//]: # (Image References)

[image0]: ./images/overview.png "Architecture"
[image1]: ./images/trajectory1.png "Trajectory1"
[image2]: ./images/trajectory2.png "Trajectory2"
[image3]: ./images/trajectory3.png "Trajectory3"
[image4]: ./images/trajectory4.png "Trajectory4"
[image5]: ./images/trajectory5.png "Trajectory5"

# Implementation of Path Planner

In this project we implemented a path planner of simulated self-driving car. We used the [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) of term 3 of Udacity Self-Driving Car Nanodegree. This simulator implements Perception, Sensor Fusion, Localization and Motion Control modules of self-driving car.
Our implementation of the path planner consists of 3 modules: 
Prediction, Behavior Planner and Trajectory Generation. The connections between different modules of path planner and simulator are visualized in the following diagram:

![alt text][image0]

Simulator calls the Path Planner every TBD millisecons. Path Planner generates trajectory of the car and sends it back to Simulator. The trajectory of the car consists of a set of waypoints. In our implementation we generated trajectories of 20 waypoints. Simulator has a motion controller that drives the car from one waypoint to the next one in 0.02 seconds. Apart from the inputs shown in the diagram, Path Planner receives also the part of the previously generates trajectory that was not used by simulator. This old trajectory is reused by Path Planner when generating a new one.

In the next sections we provide a detailed description of Prediction, Behavior Planner and Trajectory Generation modules. 

## Prediction
The goal of Prediction module is to generate an accurate prediction of the location of other cars in the road. Prediction module receives from Sensor Fusion the current location and speed of all observed cars. To generate an accurate prediction of other car's location, we used its two successive observations to estimate its acceleration. We use the following simple formula for estimating acceleration from two successive observations at times t<sub>1</sub> and t<sub>2</sub> (t<sub>1</sub><t<sub>2</sub>): 
<center>
a(t<sub>2</sub>) = (v(t<sub>2</sub>)-v(t<sub>1</sub>))/(t<sub>2</sub>-t<sub>1</sub>)
</center>  
where v and a are velocity and acceleration. Since Sensor Fusion provides v<sup>x</sup> and v<sup>y</sup> velocities in both x and y axes, we also estimated acceleration a<sup>x</sup> and a<sup>y</sup> separately in each axis. 

Notice that car's acceleration can be estimated only after receiving two observations. Hence we set car's accelration to zero when it is observed for the first time.

Prediction time is given to Prediction module by Behavior Planner module, which is described in the last section. Let t<sub>3</sub> be the prediction time and t<sub>2</sub> be the time of the last observation of the car. The prediction of location and speed to time t<sub>3</sub> is performed using the constant acceleration model. Prediction equations for the x axis are:
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
    Case A. If the previous unused trajectory has at least two waypoints, then we set the first two reference points to be the last two waypoints of the previous trajectory.    
    Case B. If the previous unused trajectory has less than two waypoints and the car has non-zero speed then the first two reference points are set to car's location before 0.02 seconds and the car's current location.   
    Case C. If the previous unused trajectory has less than two waypoints and the car has zero speed, we set the car's location to be the first reference point.  

   The next 3 reference points are set in the center of the target lane, 30, 60 and 90 meters ahead of the latest reference point.

   The next diagram shows examples of reference points when the desired lane is the same as the current one (left graph) and when the desired lane is different from the current one (right graph). In both graphs the car moves clounterclockwise and triangles are waypoints from the previously unused trajectory.The last two waypoints, used as a reference points for the new trajectory, are marked with blue. The new 3 reference points are shown with blue circles. We will also use this example in subsequent steps of trajectory generation. 

   ![alt text][image1]

2. **Convert the reference points to local coordinate system**. Spline package requires the reference points to have increasing x values. To get this property, in the above Cases A and B we convert all reference points to coordinate system centered at the second reference point and x-axis pointing in the direction from first to second reference point. This transformation is show in the following diagram:

    ![alt text][image2]

    In Case C we do the same trasformation, but with local coordinate system centered at the first reference point and the local x-axis aligned with the global one. 

3. **Generate a cubic spline that passes through the reference points**. We used a single-header file [spline package](https://github.com/ttk592/spline). To generate a spline we passed a list of reference points to `set_points` function of spline object. The next diagram shows the splines (green lines) generated for our two running examples:

    ![alt text][image3]

4. **Compute new waypoints**. Starting from the second reference point, we compute small segments along a local x axis that can be passed in 0.02 second by dringing with desired speed. The number of such segments is 20 - number of waypoints in the previously unused trajectory. For each endpoint x<sub>i</sub> of the new segments the corresponding y<sub>i</sub> value is the value of the spline at x<sub>i</sub>. The waypoints from the previously unused trajectory, along with the new points (x<sub>i</sub>,y<sub>i</sub>) are the waypoints of the new trajectory.

    The following diagram shows 3 new waypoints (top 3 blue triangles) computed along the spline. _These 3 new waypoints along with the other waypoints marked as blue triangles are the new trajectory_.

    ![alt text][image4]

5. **Convert new waypoints of a new trajectory to a global coordinate system**. Since simulator assumes that the waypoints are in global coordinate system, we convert the new waypoints (x<sub>i</sub>,y<sub>i</sub>) to the global coordinate system. The following diagram shows this transformation: 

    ![alt text][image5] 

The entire trajectory generation code can be found in `spline_trajectory` function in `trajectory_generation.cpp` file.

## Behavior Planning