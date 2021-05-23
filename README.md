

<h1 align="center">
  <br>
  <a href="https://github.com/VinBots/robots_clp"><img src="docs/assets/logo2.jpg" alt="Robotics CLP"></a>
</h1>

<h4 align="center">A Suite of Projects in Controls, Localization and Planning in Robotics </h4>
<p align="center">
  <a href="#about">About</a> •
  <a href="#learning">Learning</a> •
  <a href="#installation">Installation</a> •
  <a href="#configuration">Configuration</a> •
  <a href="#references">References</a> •
  <a href="#credits">Credits</a> •
  <a href="#going-further">Going Further</a>
</p>

---

<h1 align="center">
  <br>
  <a href="https://github.com/VinBots/robots_clp"><img src="docs/assets/clp_images2.jpg" alt="robotics"></a>
</h1>
<h5 align="center">Examples for behavioral cloning, particle filter, object detection, trajectory generation, 3D controls and 3D motion planning</h2>

## About

This repository includes a series of algorithms used in robotics, mainly self-driving vehicles and drones, for controls, localization and planning.

* **[Controls](controls/)** 
  * [PID Control](controls/PID) of a car in 2D
  * [3D Cascaded PID controller](controls/3D_cascaded_controller) for 3D  motions (positions, velocities,  accelerations and pitch, roll, yaw orientations) of a quadcopter
  * [Behavioral cloning](controls/behavioral_cloning): End-to-end learning (from pixel  to action) for predicting   steering angle of a vehicle  solely based on camera  images

* **[Localization /  Tracking](localization/)**
  *  [3D Estimation](localization/3D_estimation): Given a map, some initial localization data (like a GPS)  and at each time step, some  observation and control data,  estimation of the localization of  the vehicle
  *  [Extended Kalman Filter](localization/extended_kalman_filter): tracking a bicycle's  position and velocity using lidar and radar measurements
  *  [Basic lane line detection](localization/lane_line_detection): detecting the right and left lanes in a video of a car in a highway with basic computer vision techniques.
  *  [Advanced lane line detection](localization/advanced_lane_line_detection): detecting the right and left lanes in a video of a car in a highway with more advanced computer vision techniques
  *  [Traffic sign detection](localization/traffic_sign_classifier): classify traffic signs from the German Traffic Sign Dataset
  *  [Particle Filter](localization/particle_filter):  given a map, some initial localization information (analogous to what a GPS would provide) and at each time step observation and control data, localize a vehicle using 2D particle filter.

* **[Planning](planning/)**
  * [2D path planner](planning/path_planning): design of smooth, safe paths  for a car to follow along a 3 lane highway with traffic, using  localization, sensor fusion, and  map data
  * [3D motion planner](planning/3D_motion_planning)  for a quadcopter searching for an optimal path in a complex urban environment.

## Learning

* Deep Learning and Computer Vision (convolutional neural networks, SSD MobileNet, OpenCV)
* Particle filter, Extended  Kalman filter
* Sensor fusion
* Search
* Prediction
* Trajectory  generation
* Grid and/or  graph  representations
* Basic geometry / Basic physics  (Forces /  Moments /  Transformations)

## Installation

**1. Clone the repository**

```
git clone https://github.com/VinBots/world_models.git
```

**2. Create and activate a new virtual environment via conda**

```
conda create --name new_env python=3.6.13
conda activate new_env
```

**3. Install the required packages**

Go to the root directory and install the dependencies
```
cd world_models
pip install -r requirements.txt
```
**4. Run the algorithm**
```
python src/main.py
```

## Configuration

Please refer to each project

## References

Please refer to each project


## Credits

* Udacity [Self-Driving Car Engineer Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) program.  
* Udacity [Flying Car and Autonomous Flight Engineer Nanodegree](https://www.udacity.com/course/flying-car-nanodegree--nd787) program. 


## Going Further

* real-life applications with more noisy data