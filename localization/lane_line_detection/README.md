
<h1 align="center">Simple Lane Detection </h4>
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
  <a href="https://github.com/VinBots/world_models"><img src="examples/laneLines_thirdPass.jpg" alt="world_models"></a>
</h1>
<h5 align="center">Lane Detection on a highway</h2>

## About

When we drive, we use our eyes to decide where to go.  The lines on the road that show us where the lanes are act as our constant reference for where to steer the vehicle.  Naturally, one of the first things we would like to do in developing a self-driving car is to automatically detect lane lines using an algorithm.


### Description of the pipeline.

My pipeline consisted of 6 steps. 

* read and grayscale the image
* gaussian smoothing
* canny to detect edges
* Creating a mask
* Hough Transformation
* Display lines

I created a function draw_line_on_image that outputs an image with the drawing of identified lines

The main modifications I made are in the function draw_lines:
* the left lane lines were identified by the negative slope and the right lane lines by the positive slope
* then, I extrapolated the points at the bottom and the top the region of interests and stored them
* finally, I averaged these points

I also excluded some noise in the data by removing data points where x1=x2 or y1 = y2 and also I limited the slope to values within [-0.45;0.45] 

###  Potential shortcomings

* only works for 960 * 540 resolution
* would not work for sharp turns (abs(slope) > 0.45)
* does not adjust the region of interests according to speed and horizon
* would not work if it was snowy, rainy or dark

## Learning
* OpenCV
* gaussian smoothing
* canny to detect edges
* Creating a mask
* Hough Transformation

## Installation


**Step 1:** Set up the [CarND Term1 Starter Kit](https://github.com/udacity/CarND-Term1-Starter-Kit/blob/master/README.md) if you haven't already.

**Step 2:** Open the code in a Jupyter Notebook

## Configuration

All the parameters are located in the jupyter notebook `P1.ipynb`

## References

* This research paper goes into how to detect curves and will also help in detecting faded lanes. It uses an extended version of hough lines algorithm to detect tangents to the curve which can help you detect the curve. http://airccj.org/CSCP/vol5/csit53211.pdf
* [Cheatsheet](.../docs/assets/p1-cheat-sheet.pdf)



## Credits

* Udacity [Self-Driving Car Engineer Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) program.  


## Going Further

There are many possible improvements, for example:

* Change parameters according to the speed
* Compare series of lines to exclude significant changes of direction
* Include weather data / rain sensors / time data to adjust the model